param(
    [string]$Port='COM9',
    [ValidateRange(2,180)][int]$Seconds=5,
    [switch]$Pulse,
    [switch]$WaitForConsent,
    [switch]$SSequence,
    [ValidateRange(0,1500)][int]$InnerMv=1000,
    [ValidateRange(1,120)][double]$AfterCommandSeconds=3.5,
    [ValidateRange(-3000,3000)][int]$LeftMv=1200,
    [ValidateRange(-3000,3000)][int]$RightMv=1200,
    [ValidateRange(1,2000)][int]$DurationMs=200,
    [switch]$ProtocolCheck,
    [string]$LogPath
)
$ErrorActionPreference='Stop'
if($Pulse -and $ProtocolCheck) { throw 'Select only one action' }
if($SSequence -and (!$Pulse -or $ProtocolCheck)) { throw 'SSequence requires Pulse' }
if($SSequence) { $LeftMv=3000; $RightMv=$InnerMv; $DurationMs=2000 }
if(!$LogPath) { $LogPath=Join-Path $PSScriptRoot ('../docs/usb-odom-'+(Get-Date -Format 'yyyyMMdd-HHmmss-fff')+'.csv') }
$stream=[System.IO.File]::Open($LogPath,[System.IO.FileMode]::CreateNew,[System.IO.FileAccess]::Write,[System.IO.FileShare]::Read)
$writer=[System.IO.StreamWriter]::new($stream)
$serial=[System.IO.Ports.SerialPort]::new($Port,115200,[System.IO.Ports.Parity]::None,8,[System.IO.Ports.StopBits]::One)
$serial.ReadTimeout=50; $serial.WriteTimeout=50; $serial.NewLine="`n"
$clock=[System.Diagnostics.Stopwatch]::new()
$buffer=''; $lastRow=$null; $firstRow=$null; $sent=$false; $motionFinished=$false; $sentAt=0; $phase=0; $rows=0; $lastBeat=-100; $lastReceived=-1000; $commandSequence=0
try {
    $serial.Open(); $serial.DiscardInBuffer(); $clock.Start()
    $writer.WriteLine('host_ms,board_ms,cycle_ms,seq,accepted,drops,consent,active,fault,stop_reason,left_mv,right_mv,forward_mm,lateral_right_mm,imu_deg,gyro_dps,motor_l1_mm,motor_l2_mm,motor_l3_mm,motor_r1_mm,motor_r2_mm,motor_r3_mm,session_mm,nexus_x_mm,nexus_y_mm,nexus_h_deg,lemlib_x_mm,lemlib_y_mm,lemlib_h_deg,interlocks')
    while($clock.ElapsedMilliseconds -lt $Seconds*1000) {
        $now=$clock.ElapsedMilliseconds
        if($serial.BytesToRead -gt 0) {
            $buffer+=$serial.ReadExisting()
            $end=$buffer.IndexOf("`n")
            while($end -ge 0) {
                $line=$buffer.Substring(0,$end).TrimEnd("`r")
                $buffer=$buffer.Substring($end+1)
                if($line.StartsWith('UD1,')) {
                    $parts=$line.Split(',')
                    if($parts.Length -ne 30) { throw "Malformed UD1 frame: $line" }
                    $lastRow=$parts; $lastReceived=$now; $rows++
                    if(!$firstRow) { $firstRow=$parts }
                    $writer.WriteLine(([string]$now)+','+$line.Substring(4))
                } elseif($line.StartsWith('UD1_ERROR')) { throw $line }
                $end=$buffer.IndexOf("`n")
            }
            if($buffer.Length -gt 8192) { throw 'USB framing overflow' }
        }
        if($now -gt 1500 -and (!$lastRow -or $now-$lastReceived -gt 150)) { throw 'No fresh USBODOM telemetry' }
        if($sent -and !$motionFinished -and (!$SSequence -or $phase -eq 2) -and $lastRow -and
           [uint32]$lastRow[4] -eq $commandSequence -and $lastRow[7] -eq '0' -and $lastRow[9] -ne '0') {
            $motionFinished=$true; $serial.WriteLine('STOP')
            Write-Output ('MOTION_STOPPED reason='+$lastRow[9]+': recording only; no further pulse or heartbeat will be sent')
        }
        if(($Pulse -or $ProtocolCheck) -and $lastRow -and !$motionFinished) {
            if($lastRow[8] -ne '0') { throw 'Board safety fault; no motion command sent/continued' }
            if($now-$lastReceived -gt 80) { throw 'Telemetry stale' }
            if($now-$lastBeat -ge 30) { $serial.WriteLine('H '+$lastRow[1]); $lastBeat=$now }
            $reserve=if($SSequence){6500}else{$DurationMs+500}
            if(!$sent -and $now -ge 1000 -and $now -le [Math]::Min(30000,($Seconds*1000-$reserve)) -and (!$WaitForConsent -or $lastRow[6] -eq '1')) {
                if($Pulse -and $lastRow[6] -ne '1') { throw 'R1 consent absent; pulse not sent' }
                if($lastRow[7] -ne '0') { throw 'Robot already active; pulse not sent' }
                $commandSequence=[uint32]$lastRow[3]+1
                if($ProtocolCheck) { $l=0; $r=0 } else { $l=$LeftMv; $r=$RightMv }
                $command='P '+$lastRow[1]+' '+$commandSequence+' '+$l+' '+$r+' '+$DurationMs
                $serial.WriteLine($command); $sent=$true; $sentAt=$now; $phase=1
                Write-Output ('Sent '+$command)
            }
            if($SSequence -and $phase -eq 1) {
                if($lastRow[6] -ne '1') { throw 'R1 released: S sequence cancelled, no second pulse' }
                if($now-$sentAt -gt 200 -and [uint32]$lastRow[4] -ne $commandSequence) { throw 'First S pulse not accepted' }
                if($now-$sentAt -ge 2000 -and $lastRow[7] -eq '0' -and [uint32]$lastRow[4] -eq $commandSequence) {
                    if($lastRow[9] -ne '3') { throw 'First S pulse stopped by interlock; second pulse cancelled' }
                    $commandSequence++
                    $command='P '+$lastRow[1]+' '+$commandSequence+' '+$InnerMv+' 3000 2000'
                    $serial.WriteLine($command); $sentAt=$now; $phase=2
                    Write-Output ('Sent '+$command)
                }
            }
        }
        if($sent -and $now-$sentAt -ge $AfterCommandSeconds*1000) { break }
        if(($Pulse -or $ProtocolCheck) -and !$sent -and $now -gt 30000) { throw 'Consent window expired; no pulse sent' }
        Start-Sleep -Milliseconds 3
    }
} finally {
    if($serial.IsOpen) {
        try { $serial.WriteLine('STOP') } catch { }
        $serial.Close()
    }
    $writer.Dispose(); $serial.Dispose()
}
Write-Output ('Log: '+[System.IO.Path]::GetFullPath($LogPath))
Write-Output ('Rows: '+$rows)
if($firstRow) { Write-Output ('First: '+($firstRow -join ',')); Write-Output ('Last: '+($lastRow -join ',')) }
if($Pulse -and (!$sent -or [uint32]$lastRow[4] -ne $commandSequence)) { throw 'Pulse was NOT accepted by Brain' }
if($SSequence -and $phase -ne 2) { throw 'S sequence NOT completed' }
if($ProtocolCheck -and (!$sent -or [uint32]$lastRow[3] -ne $commandSequence -or [uint32]$lastRow[4] -eq $commandSequence)) { throw 'Expected invalid zero pulse to be consumed but NOT accepted' }
