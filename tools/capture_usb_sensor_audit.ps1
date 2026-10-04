param([string]$Port='COM9',[ValidateRange(3,60)][int]$Seconds=8)
$ErrorActionPreference='Stop'
$path=Join-Path $PSScriptRoot ('../docs/usb-sensor-audit-'+(Get-Date -Format 'yyyyMMdd-HHmmss-fff')+'.txt')
$serial=[System.IO.Ports.SerialPort]::new($Port,115200,[System.IO.Ports.Parity]::None,8,[System.IO.Ports.StopBits]::One)
$serial.ReadTimeout=100
$writer=[System.IO.StreamWriter]::new([System.IO.File]::Open($path,[System.IO.FileMode]::CreateNew,[System.IO.FileAccess]::Write,[System.IO.FileShare]::Read))
$clock=[System.Diagnostics.Stopwatch]::new()
$buffer='';$count=0;$last=0
try {
    $serial.Open();$serial.DiscardInBuffer();$clock.Start()
    while($clock.ElapsedMilliseconds -lt 1000*$Seconds) {
        $buffer+=$serial.ReadExisting();$end=$buffer.IndexOf("`n")
        while($end -ge 0) {
            $line=$buffer.Substring(0,$end).TrimEnd("`r");$buffer=$buffer.Substring($end+1)
            if($line.StartsWith('AUDIT1,') -or $line.StartsWith('AUDITS,') -or $line.StartsWith('AUDITCFG,')) {
                $writer.WriteLine($line);$count++;$last=$clock.ElapsedMilliseconds
            }
            $end=$buffer.IndexOf("`n")
        }
        if($clock.ElapsedMilliseconds -gt 2000 -and ($count -eq 0 -or $clock.ElapsedMilliseconds-$last -gt 1500)) { throw 'Audit stream missing or stale' }
        Start-Sleep -Milliseconds 10
    }
} finally {if($serial.IsOpen){$serial.Close()};$writer.Dispose();$serial.Dispose()}
Write-Output ('Log: '+[System.IO.Path]::GetFullPath($path));Write-Output ('Records: '+$count)
Get-Content -LiteralPath $path -Tail 7
