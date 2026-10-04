param(
    [string]$Port='COM9',
    [ValidateSet(-1,1)][int]$Direction=1,
    [ValidateRange(1,4)][int]$Quarters=4,
    [ValidateRange(10,120)][int]$PassiveSeconds=90,
    [string]$LogPath
)
$ErrorActionPreference='Stop'
if(!$LogPath) { $LogPath=Join-Path $PSScriptRoot ('../docs/usb-turn-'+(Get-Date -Format 'yyyyMMdd-HHmmss-fff')+'.csv') }
$stream=[System.IO.File]::Open($LogPath,[System.IO.FileMode]::CreateNew,[System.IO.FileAccess]::Write,[System.IO.FileShare]::Read)
$writer=[System.IO.StreamWriter]::new($stream)
$serial=[System.IO.Ports.SerialPort]::new($Port,115200,[System.IO.Ports.Parity]::None,8,[System.IO.Ports.StopBits]::One)
$serial.ReadTimeout=50; $serial.WriteTimeout=50; $serial.NewLine="`n"
$clock=[System.Diagnostics.Stopwatch]::new()
$buffer=''; $lastRow=$null; $firstRow=$null; $rows=0; $lastReceived=-1000; $lastBeat=-100
$sequence=0; $phase=0; $completed=0; $sentAt=0; $stoppedAt=-1; $finishedAt=-1
try {
    $serial.Open(); $serial.DiscardInBuffer(); $clock.Start()
    $writer.WriteLine('host_ms,board_ms,cycle_ms,seq,accepted,drops,consent,active,fault,stop_reason,left_mv,right_mv,forward_mm,lateral_right_mm,imu_deg,gyro_dps,motor_l1_mm,motor_l2_mm,motor_l3_mm,motor_r1_mm,motor_r2_mm,motor_r3_mm,session_mm,nexus_x_mm,nexus_y_mm,nexus_h_deg,lemlib_x_mm,lemlib_y_mm,lemlib_h_deg,interlocks')
    while($clock.ElapsedMilliseconds -lt 180000) {
        $now=$clock.ElapsedMilliseconds
        if($serial.BytesToRead -gt 0) {
            $buffer+=$serial.ReadExisting(); $end=$buffer.IndexOf("`n")
            while($end -ge 0) {
                $line=$buffer.Substring(0,$end).TrimEnd("`r"); $buffer=$buffer.Substring($end+1)
                if($line.StartsWith('UD1,')) {
                    $parts=$line.Split(','); if($parts.Length -ne 30) { throw 'Malformed telemetry' }
                    $lastRow=$parts; $lastReceived=$now; $rows++; if(!$firstRow) { $firstRow=$parts }
                    $writer.WriteLine(([string]$now)+','+$line.Substring(4))
                } elseif($line.StartsWith('UD1_ERROR')) { throw $line }
                $end=$buffer.IndexOf("`n")
            }
            if($buffer.Length -gt 8192) { throw 'USB framing overflow' }
        }
        if($now -gt 1500 -and (!$lastRow -or $now-$lastReceived -gt 150)) { throw 'No fresh telemetry' }
        if($finishedAt -ge 0) {
            if($now-$finishedAt -ge $PassiveSeconds*1000) { break }
        } elseif($lastRow) {
            if($lastRow[8] -ne '0') { throw 'Board safety fault: turn sequence cancelled' }
            if($now-$lastReceived -gt 80) { throw 'Telemetry stale' }
            if($phase -gt 0 -and $lastRow[6] -ne '1') { throw 'R1 released: no further turn will start' }
            if($now-$lastBeat -ge 30) { $serial.WriteLine('H '+$lastRow[1]); $lastBeat=$now }
            if($phase -gt 0 -and $now-$sentAt -gt 200 -and [uint32]$lastRow[4] -ne $sequence) { throw 'Turn command not accepted' }
            if($phase -gt 0 -and $stoppedAt -lt 0 -and [uint32]$lastRow[4] -eq $sequence -and $lastRow[7] -eq '0') {
                if($lastRow[9] -ne '10') { throw ('Quarter stopped before angle target: reason '+$lastRow[9]) }
                $stoppedAt=$now; $completed++
                Write-Output ('Quarter '+$completed+' stopped at heading '+$lastRow[14])
                if($completed -eq $Quarters) {
                    $finishedAt=$now; $serial.WriteLine('STOP')
                    Write-Output 'TURN_FINISHED: passive recording only; no more motor commands or heartbeat'
                }
            }
            $first=($phase -eq 0 -and $now -ge 1000 -and $now -le 30000 -and $lastRow[6] -eq '1')
            $next=($phase -gt 0 -and $phase -lt $Quarters -and $stoppedAt -ge 0 -and $now-$stoppedAt -ge 1500)
            if($finishedAt -lt 0 -and ($first -or $next)) {
                if($lastRow[7] -ne '0') { throw 'Already active' }
                $sequence=[uint32]$lastRow[3]+1
                $command='P '+$lastRow[1]+' '+$sequence+' '+(3000*$Direction)+' '+(-3000*$Direction)+' 2000'
                $serial.WriteLine($command); $phase++; $sentAt=$now; $stoppedAt=-1
                Write-Output ('Sent '+$command)
            }
            if($phase -eq 0 -and $now -gt 30000) { throw 'R1 consent window expired; no turn sent' }
        }
        Start-Sleep -Milliseconds 3
    }
} finally {
    if($serial.IsOpen) { try { $serial.WriteLine('STOP') } catch {}; $serial.Close() }
    $writer.Dispose(); $serial.Dispose()
}
Write-Output ('Log: '+[System.IO.Path]::GetFullPath($LogPath))
Write-Output ('Rows: '+$rows+'; completed quarters: '+$completed)
if($firstRow) { Write-Output ('First: '+($firstRow -join ',')); Write-Output ('Last: '+($lastRow -join ',')) }
if($completed -ne $Quarters) { throw 'Requested turn NOT completed' }
