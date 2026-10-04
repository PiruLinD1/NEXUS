#include "robot.hpp"
#include "robot-config.hpp"
#include "robot/calibration_support.hpp"
#include "nexus/automatic_calibration.hpp"
#include "nexus/calibration_routine.hpp"
#include "nexus/pod_offset_calibration.hpp"
#include "pros/llemu.hpp"
#include <atomic>
#include <cstdarg>
#include <cstdio>

namespace robot {
namespace {
using namespace nexus;
using namespace nexus::calibration;
enum class Phase { idle, menu, stationary, armed, geometryStationary, turning, review,
                   podArmed, podRunning, podReview, result, leaving };
constexpr std::uint32_t connectedFlag = 1u << 16;
constexpr std::uint32_t bit(Button button) {
    return 1u << (static_cast<int>(button) - static_cast<int>(L1));
}
std::atomic<std::uint32_t> inputBits{0}, inputTime{0}, abortEpoch{0};
std::atomic<bool> active{false}, resetDriverRequested{false};
std::unique_ptr<pros::Task> calibrationTask;

bool practiceEnabled() {
    return !pros::competition::is_disabled() && !pros::competition::is_autonomous()
        && !pros::competition::is_connected();
}
double seconds() { return static_cast<double>(pros::micros()) * 1e-6; }
bool stopOutputs() {
    // This task persists across competition modes; never call robot::stop()
    // here, because that also requests cancellation of the calibration UI.
    if (!ready() || !chassis().stopPractice() || !practiceEnabled()) return false;
    lift(0); braccio(0); intake(false);
    return true;
}

class CalibrationService {
public:
    void tick() {
        const auto tickTime = pros::millis();
        const double now = seconds();
        const auto bits = inputBits.load();
        const auto presses = bits & ~previousBits_;
        previousBits_ = bits;
        const bool inputLive = (bits & connectedFlag) && tickTime - inputTime.load() <= 100;
        const bool allowed = ready() && practiceEnabled() && inputLive;
        const auto abort = abortEpoch.load();
        if (phase_ != Phase::idle && (!allowed || abort != observedAbort_)) {
            // A new competition callback may already own the motors. Release only
            // our lease; do not cancel that callback's newly started motion.
            exit(practiceEnabled() && abort == observedAbort_);
            observedAbort_ = abort;
            return;
        }
        observedAbort_ = abort;
        if (phase_ == Phase::idle) {
            const bool chord = allowed && (bits & bit(L1)) && (bits & bit(R1));
            if (!chord) chordStart_ = 0;
            else if (!chordStart_) chordStart_ = tickTime;
            else if (tickTime - chordStart_ >= 1000) {
                active = true;
                if (!stopOutputs()) { exit(false); return; }
                context_ = detail::calibrationContext();
                phase_ = Phase::menu;
                waitForRelease_ = true;
                lastRender_ = 0;
            }
            return;
        }
        if (waitForRelease_) {
            if (!(bits & (bit(L1) | bit(R1)))) waitForRelease_ = false;
        } else if (phase_ == Phase::menu) {
            if (presses & bit(B)) { requestExit(); return; }
            if (presses & bit(A)) beginStationary(now, false);
            else if (presses & bit(X)) { phase_ = Phase::armed; lastRender_ = 0; }
            else if (presses & bit(Y)) { phase_ = Phase::podArmed; lastRender_ = 0; }
        } else if (phase_ == Phase::podArmed) {
            if (presses & bit(B)) phase_ = Phase::menu;
            else if (presses & bit(R1)) beginPodOffsets();
        } else if (phase_ == Phase::podRunning) {
            if (bits & bit(B)) fail("Prova pod annullata");
            else {
                const auto sample = chassis().diagnostics().sensors;
                const auto output = podRoutine_.update(sample, seconds(), (bits & bit(R1)) != 0,
                                                        (presses & bit(A)) != 0);
                if (output.stage == PodOffsetStage::aborted) fail(podOffsetFailureName(output.failure));
                else if (output.stage == PodOffsetStage::complete) {
                    stopLease();
                    phase_ = Phase::podReview; lastRender_ = 0;
                    const auto& fit = podRoutine_.fit().result();
                    std::printf("PODOFFSET,forward_mm=%.3f,lateral_mm=%.3f,F1_mm=%.3f,L1_mm=%.3f,F2_mm=%.3f,L2_mm=%.3f,heading1_deg=%.3f,heading2_deg=%.3f,home_deg=%.3f,consistent=%u\n",
                        fit.forwardOffset / millimeter, fit.lateralOffset / millimeter,
                        fit.individual[0][0] / millimeter, fit.individual[0][1] / millimeter,
                        fit.individual[1][0] / millimeter, fit.individual[1][1] / millimeter,
                        degrees(fit.angles[0]), degrees(fit.angles[1]), degrees(output.heading),
                        static_cast<unsigned>(fit.consistent));
                } else if (!chassis().calibrationVoltage(lease_, output.voltage.left, output.voltage.right))
                    fail("Prova pod: comando revocato");
            }
        } else if (phase_ == Phase::podReview) {
            if (presses & bit(B)) { requestExit(); return; }
            if (presses & bit(A)) phase_ = Phase::menu;
            else if ((presses & bit(X)) && podRoutine_.fit().result().consistent) {
                const auto& fit = podRoutine_.fit().result();
                auto profile = context_.profile;
                profile.forwardOffset = fit.forwardOffset;
                profile.lateralOffset = fit.lateralOffset;
                profile.calibratedMask |= forwardOffsetField | lateralOffsetField;
                const auto applied = detail::applyCalibrationProfile(profile, false);
                if (applied.applied) result("Offset applicati solo in RAM");
                else fail("Resta fermo: offset non applicati");
            }
        } else if (phase_ == Phase::armed) {
            if (presses & bit(B)) phase_ = Phase::menu;
            else if (presses & bit(R1)) beginStationary(now, true);
        } else if (phase_ == Phase::stationary || phase_ == Phase::geometryStationary) {
            const bool geometry = phase_ == Phase::geometryStationary;
            if ((bits & bit(B)) || (geometry && !(bits & bit(R1)))) {
                fail("Annullata: motori fermi");
            } else {
                const auto sample = chassis().diagnostics().sensors;
                if (!fresh(sample, seconds())) fail("Dati sensori assenti/vecchi");
                else if (sample.timestamp != lastSample_) {
                    lastSample_ = sample.timestamp;
                    if (!stationary_.add(sample)) fail("Resta fermo: dati rifiutati");
                }
                if ((phase_ == Phase::stationary || phase_ == Phase::geometryStationary)
                    && now - phaseStarted_ >= 5.0) {
                    stationaryResult_ = stationary_.finish();
                    if (!stationaryResult_.quality.accepted) fail(fitFailureName(stationaryResult_.quality.failure));
                    else if (!chassis().setGyroBias(stationaryResult_.gyroBias, stationaryResult_.gyroBiasVariance))
                        fail("Bias IMU non applicato");
                    else if (!geometry) result("Bias aggiornato (questa accensione)");
                    else beginTurns();
                }
            }
        } else if (phase_ == Phase::turning) {
            const auto sample = chassis().diagnostics().sensors;
            const bool permit = (bits & bit(R1)) && !(bits & bit(B));
            const auto output = routine_.update(sample, seconds(), permit);
            if (output.stage == CalibrationStage::aborted) fail(calibrationFailureName(output.failure));
            else {
                if (sample.timestamp != lastSample_) {
                    lastSample_ = sample.timestamp;
                    if (!rotation_.add(sample)) fail("Geometria: sensori incoerenti");
                }
                if (phase_ == Phase::turning) {
                    if (output.stage == CalibrationStage::complete) {
                        stopLease();
                        geometry_ = rotation_.finish();
                        if (!geometry_.accepted) fail(fitFailureName(geometry_.failure));
                        else { phase_ = Phase::review; lastRender_ = 0; }
                    } else if (!chassis().calibrationVoltage(lease_, output.voltage.left, output.voltage.right))
                        fail("Comando revocato: motori fermi");
                }
            }
        } else if (phase_ == Phase::review) {
            if (presses & bit(B)) { result("Risultati scartati"); }
            else if (presses & (bit(A) | bit(X))) {
                auto profile = context_.profile;
                if (!geometry_.applyTo(profile)) fail("Profilo rifiutato");
                else {
                    const bool persist = (presses & bit(A)) != 0;
                    const auto applied = detail::applyCalibrationProfile(profile, persist);
                    if (!applied.applied) fail("Profilo non applicato");
                    else if (!persist) result("Applicato solo in RAM");
                    else if (applied.storage == StorageStatus::ok) result("Applicato e salvato su microSD");
                    else result("Applicato RAM; salvataggio fallito");
                }
            }
        } else if (phase_ == Phase::result) {
            if (presses & bit(B)) { requestExit(); return; }
            if (presses & bit(A)) phase_ = Phase::menu;
        } else if (phase_ == Phase::leaving) {
            // B is also the lift-up binding. Do not hand it back still held.
            if (!(bits & 0xfffu)) { exit(); return; }
        }
        if (tickTime - lastRender_ >= 100) { render(now); lastRender_ = tickTime; }
    }
private:
    Phase phase_ = Phase::idle;
    std::uint32_t previousBits_ = 0, chordStart_ = 0, observedAbort_ = 0, lastRender_ = 0, lease_ = 0;
    bool waitForRelease_ = false;
    double phaseStarted_ = 0, lastSample_ = -1;
    std::array<char, 64> message_{};
    detail::CalibrationContext context_{};
    StationaryCalibration stationary_; // Persistent storage, not callback stack.
    RotationCalibration rotation_;
    GeometryRoutine routine_;
    StationaryResult stationaryResult_{};
    AutomaticGeometryResult geometry_{};
    PodOffsetRoutine podRoutine_;

    static bool fresh(const SensorSample& sample, double now) {
        return std::isfinite(sample.timestamp) && sample.timestamp > 0
            && now >= sample.timestamp && now - sample.timestamp <= 0.10;
    }
    void beginPodOffsets() {
        if (!stopOutputs()) { exit(false); return; }
        context_ = detail::calibrationContext();
        if (context_.imuCalibrationFailed) { fail("IMU: riavvia fermo per calibrare"); return; }
        const auto sample = chassis().diagnostics().sensors;
        if (!podRoutine_.start(sample, seconds(), context_.estimator)) {
            fail("Servono due pod e IMU nativa validi"); return;
        }
        lease_ = chassis().beginCalibration(CalibrationSensors::trackingPods);
        if (!lease_) { fail("Prova pod non consentita"); return; }
        phase_ = Phase::podRunning; lastRender_ = 0;
    }
    void beginStationary(double now, bool geometry) {
        if (!stopOutputs()) { exit(false); return; }
        context_ = detail::calibrationContext();
        if (context_.imuCalibrationFailed) { fail("IMU: riavvia fermo per calibrare"); return; }
        if (context_.estimator.nativeImuHeading) {
            if (!geometry) result("IMU nativa: calibrazione solo all'avvio");
            else {
                stationaryResult_ = {};
                beginTurns();
            }
            return;
        }
        stationary_.reset(context_.estimator.gyroScale);
        lastSample_ = -1;
        phaseStarted_ = now;
        phase_ = geometry ? Phase::geometryStationary : Phase::stationary;
        lastRender_ = 0;
    }
    void beginTurns() {
        const auto sample = chassis().diagnostics().sensors;
        const double now = seconds();
        const double bias = context_.estimator.nativeImuHeading ? 0 : stationaryResult_.gyroBias;
        rotation_.reset({context_.estimator.forwardScale, context_.estimator.lateralScale,
                         context_.estimator.gyroScale, bias,
                         config::forwardPodPort != 0, config::lateralPodPort != 0});
        if (!rotation_.add(sample) || !routine_.start(sample, now, context_.estimator.gyroScale, bias)) {
            fail("Avvio rotazioni rifiutato"); return;
        }
        lease_ = chassis().beginCalibration();
        if (!lease_) { fail("Calibrazione non consentita"); return; }
        phase_ = Phase::turning;
        lastSample_ = sample.timestamp;
        lastRender_ = 0;
    }
    void result(const char* message) {
        stopLease();
        if (practiceEnabled()) { lift(0); braccio(0); intake(false); }
        std::snprintf(message_.data(), message_.size(), "%s", message);
        phase_ = Phase::result; lastRender_ = 0;
    }
    void fail(const char* message) { result(message); }
    void stopLease() {
        if (lease_ && ready()) chassis().endCalibration(lease_);
        lease_ = 0;
    }
    void requestExit() {
        stopLease();
        phase_ = Phase::leaving;
        lastRender_ = 0;
    }
    void exit(bool stopMechanisms = true) {
        stopLease();
        if (stopMechanisms && practiceEnabled()) { lift(0); braccio(0); intake(false); }
        phase_ = Phase::idle; lease_ = 0; chordStart_ = 0;
        resetDriverRequested = true;
        active = false;
    }
    struct Screen {
        std::array<std::array<char, 64>, 8> lines{};
        __attribute__((format(printf, 3, 4)))
        void put(std::size_t line, const char* fmt, ...) {
            va_list args;
            va_start(args, fmt);
            std::vsnprintf(lines[line].data(), lines[line].size(), fmt, args);
            va_end(args);
        }
    };
    void render(double now) {
        Screen screen;
        screen.put(0, "CALIBRAZIONE - modalita laboratorio");
        if (phase_ == Phase::menu) {
            screen.put(1, "Rilascia L1 e R1");
            screen.put(2, "%s", context_.estimator.nativeImuHeading
                ? "A: info IMU nativa (bias extra disattivato)"
                : "A: bias IMU, robot fermo 5 secondi");
            screen.put(3, "X: offset pod + carreggiata");
            screen.put(4, "B: esci e torna alla guida");
            screen.put(5, "Y: offset SOLO POD+IMU, due riferimenti");
            screen.put(6, "Geometria: area libera, ruote a terra");
            screen.put(7, "Le scale assolute richiedono misure");
        } else if (phase_ == Phase::podArmed) {
            screen.put(0, "OFFSET POD | riferimento sul telaio");
            screen.put(1, "Segna il centro geometrico e il pavimento");
            screen.put(2, "Due soste: circa +90 e -90 gradi");
            screen.put(3, "A ogni sosta ricentra il punto sul segno");
            screen.put(4, "Mantieni l'orientamento della sosta");
            screen.put(5, "Tieni R1 per iniziare | B indietro");
            screen.put(6, "R1 rilasciato in rotazione: STOP");
            screen.put(7, "Encoder motore non usati nella prova");
        } else if (phase_ == Phase::podRunning) {
            const auto& output = podRoutine_.output();
            screen.put(0, "OFFSET POD | fase %u/3", output.leg + 1);
            screen.put(1, "Heading %+.1f -> %+.0f deg", degrees(output.heading), degrees(output.target));
            screen.put(2, "Pod F %+.1f L %+.1f mm", podRoutine_.fit().forwardTravel() / millimeter,
                       podRoutine_.fit().lateralTravel() / millimeter);
            if (output.stage == PodOffsetStage::align) {
                screen.put(3, "FERMO: rilascia R1");
                screen.put(4, "Ricentra il punto telaio sul segno");
                screen.put(5, "Mantieni circa %+.0f deg, NON tornare a 0", degrees(output.target));
                screen.put(6, "Fermo sul segno: premi A");
            } else if (output.stage == PodOffsetStage::ready) {
                screen.put(3, "Riferimento registrato");
                screen.put(4, "Tieni nuovamente R1 per continuare");
                screen.put(5, "%s", output.leg == 2 ? "Ultima rotazione: ritorno heading iniziale" : "Prossima rotazione: senso antiorario");
            } else {
                screen.put(3, "%s", output.stage == PodOffsetStage::settling ? "ARRESTO: attendi, tieni R1" : "ROTAZIONE: tieni R1");
                screen.put(4, "Obiettivo %.0f deg/s | max %.1f V",
                    degrees(output.leg == 2 ? PodOffsetRoutine::homeSpeed : PodOffsetRoutine::targetSpeed),
                    PodOffsetRoutine::maxVoltage);
                screen.put(5, "Misurata %.0f deg/s | comando %.1f V", degrees(output.speed), output.voltage.left);
                screen.put(6, "Rilascia R1: arresto e annulla");
            }
            screen.put(7, "B: annulla");
        } else if (phase_ == Phase::podReview) {
            const auto& fit = podRoutine_.fit().result();
            screen.put(0, "OFFSET POD | RISULTATI (mm)");
            screen.put(1, "Avanti X: %+.2f mm", fit.forwardOffset / millimeter);
            screen.put(2, "Laterale Y: %+.2f mm", fit.lateralOffset / millimeter);
            screen.put(3, "Differenza prove F %.1f L %.1f mm", fit.forwardDifference / millimeter,
                       fit.lateralDifference / millimeter);
            screen.put(4, "Heading finale: %+.2f deg", degrees(podRoutine_.output().heading));
            screen.put(5, "%s", fit.consistent ? "Stima concorde: verificare su percorso" : "Prove discordanti: ripetere riferimenti");
            screen.put(6, "%s", fit.consistent ? "X: applica solo in RAM (facoltativo)" : "Valori mostrati, applicazione disattivata");
            screen.put(7, "A: menu | B: esci");
        } else if (phase_ == Phase::armed) {
            screen.put(1, "Il robot fara un giro per verso");
            screen.put(2, "%s", context_.estimator.nativeImuHeading
                ? "IMU nativa: nessuna misura bias aggiuntiva"
                : "Prima: 5 secondi completamente fermo");
            screen.put(3, "TIENI R1 per tutta la procedura");
            screen.put(4, "Rilascia R1 o premi B per fermare");
            screen.put(6, "Verifica pod e ruote a terra");
            screen.put(7, "X/Y e orientamento non sono azzerati");
        } else if (phase_ == Phase::stationary || phase_ == Phase::geometryStationary) {
            screen.put(1, "FERMO: misura bias IMU %.1f / 5s", std::min(5.0, now - phaseStarted_));
            screen.put(2, "Non toccare o muovere il robot");
            if (phase_ == Phase::geometryStationary) {
                screen.put(3, "Tieni R1: dopo iniziano le rotazioni");
                screen.put(4, "Rilascia R1 o B: annulla");
            } else screen.put(4, "B: annulla");
        } else if (phase_ == Phase::turning) {
            const auto output = routine_.output();
            screen.put(1, "ROTAZIONI - tieni premuto R1");
            screen.put(2, "Orario %.0f / 360 deg", degrees(output.clockwiseRadians));
            screen.put(3, "Antiorario %.0f / 360 deg", degrees(output.counterclockwiseRadians));
            screen.put(4, "Rilascia R1 o B: arresto");
            screen.put(5, "Velocita obiettivo: %.0f deg/s", degrees(GeometryRoutine::targetAngularSpeed));
            if (context_.estimator.nativeImuHeading)
                screen.put(6, "IMU nativa: bias extra disattivato");
            else screen.put(6, "Bias misurato %.3f deg/s", degrees(stationaryResult_.gyroBias));
        } else if (phase_ == Phase::review) {
            screen.put(1, "Risultati coerenti nei due versi");
            if (config::forwardPodPort) screen.put(2, "Offset pod avanti X: %.1f mm", geometry_.forwardOffset.value / millimeter);
            if (config::lateralPodPort) screen.put(3, "Offset pod laterale Y: %.1f mm", geometry_.lateralOffset.value / millimeter);
            screen.put(4, "Carreggiata effettiva: %.1f mm", geometry_.trackWidth.value / millimeter);
            screen.put(5, "A: applica e salva su microSD");
            screen.put(6, "X: applica solo per questa accensione");
            screen.put(7, "B: scarta (nessuna modifica geometria)");
        } else if (phase_ == Phase::result) {
            screen.put(1, "%s", message_.data());
            screen.put(3, "A: menu calibrazione");
            screen.put(4, "B: torna alla guida");
            screen.put(6, "Convalida su movimenti misurati");
        } else if (phase_ == Phase::leaving) {
            screen.put(1, "Rilascia i tasti per tornare alla guida");
        }
        for (int i = 0; i < 8; ++i) pros::lcd::print(i, "%s", screen.lines[i].data());
    }
};
CalibrationService service; // Fixed-capacity collectors kept off task stacks.
} // namespace

bool calibrationActive() {
    if (active.load()) return true;
    if (resetDriverRequested.exchange(false)) { resetDriver(); return true; }
    return false;
}
namespace detail {
void publishCalibrationInput(std::uint32_t buttons, bool connected) {
    inputBits = buttons | (connected ? connectedFlag : 0);
    inputTime = pros::millis();
}
void abortCalibration() { ++abortEpoch; }
bool calibrationScreenActive() { return active.load(); }
void startCalibrationService() {
    if (calibrationTask) return;
    calibrationTask = std::make_unique<pros::Task>([] {
        std::uint32_t wake = pros::millis();
        for (;;) {
            service.tick();
            if (pros::millis() - wake >= 10) wake = pros::millis();
            pros::Task::delay_until(&wake, 10);
        }
    }, TASK_PRIORITY_DEFAULT - 1, 32768, "robot-calibration");
}
} // namespace detail
} // namespace robot
