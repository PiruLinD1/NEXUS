param(
    [string]$Port='COM9',
    [ValidateRange(3,600)][int]$Seconds=180,
    [switch]$UntilArrival,
    [string]$LogPath
)
$ErrorActionPreference='Stop'
if(!$LogPath) { $LogPath=Join-Path $PSScriptRoot ('../docs/usb-passive-'+(Get-Date -Format 'yyyyMMdd-HHmmss-fff')+'.csv') }
$fields='board_ms,cycle_ms,sample_seq,marker,marker_ok,controller_connected,valid_mask,drops,forward_raw_cd,lateral_raw_cd,forward_mm,lateral_right_mm,imu_deg,gyro_x_dps,gyro_y_dps,gyro_z_dps,accel_x_g,accel_y_g,accel_z_g,pitch_deg,roll_deg,motor_l1_rev,motor_l2_rev,motor_l3_rev,motor_r1_rev,motor_r2_rev,motor_r3_rev,f_before_us,f_after_us,l_before_us,l_after_us,g_before_us,g_after_us,drive_before_us,drive_after_us'
$count=$fields.Split(',').Length+1
$stream=[System.IO.File]::Open($LogPath,[System.IO.FileMode]::CreateNew,[System.IO.FileAccess]::Write,[System.IO.FileShare]::Read)
$writer=[System.IO.StreamWriter]::new($stream)
$serial=[System.IO.Ports.SerialPort]::new($Port,115200,[System.IO.Ports.Parity]::None,8,[System.IO.Ports.StopBits]::One)
$serial.ReadTimeout=50
$clock=[System.Diagnostics.Stopwatch]::new()
$buffer='';$last=$null;$first=$null;$received=-1000;$rows=0;$marker=0;$arrival=-1
try {
    $serial.Open();$serial.DiscardInBuffer();$clock.Start();$writer.WriteLine('host_ms,'+$fields)
    while($clock.ElapsedMilliseconds -lt $Seconds*1000) {
        $now=$clock.ElapsedMilliseconds
        if($serial.BytesToRead -gt 0) {
            $buffer+=$serial.ReadExisting();$end=$buffer.IndexOf("`n")
            while($end -ge 0) {
                $line=$buffer.Substring(0,$end).TrimEnd("`r");$buffer=$buffer.Substring($end+1)
                if($line.StartsWith('UP1,')) {
                    $parts=$line.Split(',');if($parts.Length -ne $count) {throw "Malformed passive frame: expected $count fields"}
                    if(!$first) {$first=$parts;if($UntilArrival -and [int]$parts[4] -ne 0) {throw 'Markers already used: restart passive firmware before a new test'}}
                    $last=$parts;$received=$now;$rows++;$writer.WriteLine(([string]$now)+','+$line.Substring(4))
                    if([int]$parts[4] -ne $marker) {
                        $marker=[int]$parts[4];Write-Output ('MARK '+$marker+' board_ms='+$parts[1]+' heading='+$parts[13])
                        $writer.Flush()
                        if($marker -eq 3) {$arrival=$now}
                        if($marker -gt 3 -and $UntilArrival) {throw 'Too many markers: preserve log and inspect before fitting'}
                    }
                } elseif($line.StartsWith('UP1_ERROR')) {throw $line}
                $end=$buffer.IndexOf("`n")
            }
            if($buffer.Length -gt 16384) {throw 'Framing overflow'}
        }
        if($now -gt 1500 -and (!$last -or $now-$received -gt 200)) {throw 'No fresh USBODOM 5 telemetry'}
        if($UntilArrival -and $arrival -ge 0 -and $now-$arrival -ge 2500) {break}
        Start-Sleep -Milliseconds 3
    }
} finally {
    if($serial.IsOpen) {$serial.Close()};$writer.Dispose();$serial.Dispose()
}
Write-Output ('Log: '+[System.IO.Path]::GetFullPath($LogPath));Write-Output ('Rows: '+$rows)
if($first) {Write-Output ('First: '+($first -join ','));Write-Output ('Last: '+($last -join ','))}
if($UntilArrival -and $marker -ne 3) {throw 'Arrival marker not received; log retained, test incomplete'}
