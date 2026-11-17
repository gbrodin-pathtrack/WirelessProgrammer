<#
Connects to the wireless programmer's MCUboot serial recovery when it's plugged in, lists the image it holds,
optionally uploads a new one, then resets the WP so the app starts, unless -Stay is given. Unplug and plug in again to
repeat, Ctrl+C to stop.

MCUboot only waits CONFIG_BOOT_SERIAL_WAIT_FOR_DFU_TIMEOUT (3 s) after a reset for a command, so this has to be
running before the WP is plugged in. As soon as a new COM port appears it is opened and MCUboot is sent an SMP echo
every 100 ms until it answers, which keeps it in recovery. Everything after that is done with
nrfutil mcu-manager (nrfutil install mcu-manager), as MCUboot then waits as long as needed.

The WP's bridge only passes data on while DTR is asserted, so DTR is on; that is only safe while neither DTR nor RTS is
wired to the WP's reset.

Run with: powershell -ExecutionPolicy Bypass -File mcuboot_connect.ps1 [-Firmware <signed and encrypted .bin>] [-Stay]
#>
param(
    # The signed and encrypted app from release/, uploaded over the current one
    [string]$Firmware,
    [switch]$Stay,
    # MCUboot's rate, set in sysbuild/mcuboot.overlay, not the app's
    [int]$BaudRate = 115200,
    [int]$PollMs = 10,
    [int]$OpenTimeoutMs = 5000,
    # Longer than MCUboot's 3 s window, so a missed window is reported rather than cut short
    [int]$ConnectTimeoutMs = 5000
)

function Write-Timed([string]$message) {
    Write-Host ("{0}  {1}" -f (Get-Date -Format "HH:mm:ss.fff"), $message)
}

function Get-Ports {
    return @([System.IO.Ports.SerialPort]::GetPortNames() | Sort-Object -Unique)
}

# CRC-16/XMODEM (polynomial 0x1021, starting at 0), MCUboot's crc16_itu_t
function Get-Crc16([byte[]]$data) {
    $crc = 0
    foreach ($b in $data) {
        $crc = $crc -bxor ([int]$b -shl 8)
        for ($i = 0; $i -lt 8; $i++) {
            if ($crc -band 0x8000) {
                $crc = (($crc -shl 1) -bxor 0x1021) -band 0xFFFF
            } else {
                $crc = ($crc -shl 1) -band 0xFFFF
            }
        }
    }
    return $crc
}

# An SMP echo request of {"d": "WP"} as a serial frame: 0x06 0x09, then base64 of the total length, the 8-byte header
# (write, group 0, command 0), the CBOR payload and the CRC, all big-endian, then a newline. A newline first ends any
# partial line MCUboot has buffered.
function New-EchoFrame {
    $payload = [byte[]]@(0xA1, 0x61, 0x64, 0x62, 0x57, 0x50)
    $packet = [byte[]](@(2, 0, 0, $payload.Length, 0, 0, 0, 0) + $payload)
    $crc = Get-Crc16 $packet
    $total = $packet.Length + 2
    $body = [byte[]](@(0, $total) + $packet + @(($crc -shr 8), ($crc -band 0xFF)))
    $line = [char]0x0A + [char]0x06 + [char]0x09 + [Convert]::ToBase64String($body) + [char]0x0A
    return [System.Text.Encoding]::GetEncoding(28591).GetBytes($line)
}

# Runs one nrfutil mcu-manager serial command on the port, returning whether it succeeded
function Invoke-McuManager([string]$port, [string[]]$arguments) {
    & nrfutil mcu-manager serial @arguments --serial-port "port=$port,baud_rate=$BaudRate"
    return $LASTEXITCODE -eq 0
}

if (-not (Get-Command nrfutil -ErrorAction SilentlyContinue)) {
    Write-Timed "nrfutil not found, install it and run: nrfutil install mcu-manager"
    exit 1
}
if ($Firmware -and -not (Test-Path -LiteralPath $Firmware -PathType Leaf)) {
    Write-Timed "Firmware not found: $Firmware"
    exit 1
}

$SmpReplyStart = [string][char]0x06 + [char]0x09
$EchoFrame = New-EchoFrame

$known = Get-Ports
Write-Timed ("Existing ports: {0}" -f $(if ($known.Count) { $known -join ", " } else { "none" }))

while ($true) {
    Write-Timed "Waiting for a new COM port, plug in the device"
    $port = $null
    while (-not $port) {
        $ports = Get-Ports
        $port = $ports | Where-Object { $known -notcontains $_ } | Select-Object -First 1
        # Ports that disappeared are dropped, so plugging the same device in again is seen as new
        $known = $ports | Where-Object { $_ -ne $port }
        if (-not $port) {
            Start-Sleep -Milliseconds $PollMs
        }
    }
    $sincePort = [System.Diagnostics.Stopwatch]::StartNew()
    Write-Timed "$port appeared"

    $serial = New-Object System.IO.Ports.SerialPort $port, $BaudRate, ([System.IO.Ports.Parity]::None), 8, ([System.IO.Ports.StopBits]::One)
    $serial.DtrEnable = $true
    $serial.RtsEnable = $false
    # Byte for byte, so binary data from the app can't upset the frame search
    $serial.Encoding = [System.Text.Encoding]::GetEncoding(28591)

    # The port can be listed a little before it can be opened, so keep trying
    $opened = $false
    $openError = $null
    while (-not $opened -and $sincePort.ElapsedMilliseconds -lt $OpenTimeoutMs) {
        try {
            $serial.Open()
            $opened = $true
        } catch {
            $openError = $_.Exception
            Start-Sleep -Milliseconds $PollMs
        }
    }

    $inRecovery = $false
    if (-not $opened) {
        # PowerShell wraps the SerialPort exception, so report the original
        while ($openError.InnerException) {
            $openError = $openError.InnerException
        }
        Write-Timed ("{0} couldn't be opened within {1} ms: {2}: {3}" -f $port, $OpenTimeoutMs, $openError.GetType().Name, $openError.Message)
        if ($openError -is [System.UnauthorizedAccessException]) {
            Write-Timed "Another program has the port open, close it and try again"
        }
    } else {
        Write-Timed ("{0} opened {1} ms after it appeared, sending SMP echo to MCUboot" -f $port, $sincePort.ElapsedMilliseconds)

        # MCUboot may not be listening yet, so keep asking until it answers or its window must have passed. Any SMP
        # reply will do: the app never sends one.
        $received = ""
        $attempts = 0
        while (-not $inRecovery -and $sincePort.ElapsedMilliseconds -lt $ConnectTimeoutMs) {
            $attempts++
            $serial.Write($EchoFrame, 0, $EchoFrame.Length)
            $sinceEcho = [System.Diagnostics.Stopwatch]::StartNew()
            while (-not $inRecovery -and $sinceEcho.ElapsedMilliseconds -lt 100) {
                $received += $serial.ReadExisting()
                $inRecovery = $received.Contains($SmpReplyStart)
                Start-Sleep -Milliseconds $PollMs
            }
        }

        if ($inRecovery) {
            Write-Timed ("MCUboot answered {0} ms after the port appeared ({1} attempts), it stays in recovery until reset" -f
                         $sincePort.ElapsedMilliseconds, $attempts)
            # Replies to the other echoes would otherwise be left for nrfutil to read as the answer to its first request
            Start-Sleep -Milliseconds 200
            $serial.DiscardInBuffer()
        } else {
            Write-Timed ("No reply from MCUboot within {0} ms of the port appearing ({1} attempts). Its window was missed and the app started, or the WP isn't running a build with MCUboot." -f $ConnectTimeoutMs, $attempts)
        }
        $serial.Close()
    }
    $serial.Dispose()

    if ($inRecovery) {
        $ok = Invoke-McuManager $port @("image-list", "--timeout", "3")
        if ($ok -and $Firmware) {
            Write-Timed "Uploading $Firmware"
            $ok = Invoke-McuManager $port @("image-upload", "--firmware", $Firmware, "--image-number", "0")
            if ($ok) {
                # MCUboot decrypts the image in place after acknowledging the last chunk, before it reads this request
                $ok = Invoke-McuManager $port @("image-list", "--timeout", "10")
            }
        }

        if (-not $ok) {
            Write-Timed "nrfutil failed, see its output above. MCUboot is still in recovery, so unplug and try again"
        } elseif ($Stay) {
            Write-Timed "Left in recovery (-Stay), unplug or reset the WP to start the app"
        } else {
            # MCUboot replies before it resets, so a failure here is reported rather than assumed
            if (Invoke-McuManager $port @("reset", "--timeout", "3")) {
                Write-Timed "Reset sent, the app should now start"
            } else {
                Write-Timed "Reset failed, unplug the WP to start the app"
            }
        }
    }

    Write-Timed "Unplug the device to try again"
    while ((Get-Ports) -contains $port) {
        Start-Sleep -Milliseconds 100
    }
    $known = Get-Ports
    Write-Timed "$port removed"
}
