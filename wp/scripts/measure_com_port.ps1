<#
Measures how soon a newly plugged-in COM port can be used, to size a boot-time wait for MCUboot serial recovery.

Start this first, then plug in the wireless programmer. As soon as a new COM port appears it is opened and 0x55 bytes
sent for half a second, which the WP ignores. With the WP built with WP_TEST_FIRST_PC_BYTE=1 it sends back how long
after it started the first byte arrived, the byte's value, and its reset cause, which this script prints, so no
debugger is needed. The WP must be powered from the same USB plug as its UART bridge, so plugging in starts it.
Unplug and plug in again to repeat, Ctrl+C to stop.

The WP's bridge only passes data on while DTR is asserted, so DTR is on unless -NoDtr is given. -Rts asserts RTS too.
Both are only safe while neither line is wired to the WP's reset.

Run with: powershell -ExecutionPolicy Bypass -File measure_com_port.ps1 [-BaudRate 57600] [-PollMs 10] [-NoDtr] [-Rts]
#>
param(
    [int]$BaudRate = 57600,
    [int]$PollMs = 10,
    [int]$OpenTimeoutMs = 5000,
    # The WP checks for the first byte once per pairing attempt, about every 3 s, before replying
    [int]$ReplyTimeoutMs = 5000,
    [switch]$NoDtr,
    [switch]$Rts
)

# Zephyr's hwinfo reset cause flags
$ResetCauses = [ordered]@{
    0x00001 = "pin"; 0x00002 = "software"; 0x00004 = "brownout"; 0x00008 = "power-on"; 0x00010 = "watchdog";
    0x00020 = "debug"; 0x00040 = "security"; 0x00080 = "low power wake"; 0x00100 = "CPU lockup"; 0x00200 = "parity";
    0x00400 = "PLL"; 0x00800 = "clock"; 0x01000 = "hardware"; 0x02000 = "user"; 0x04000 = "temperature";
    0x08000 = "bootloader"; 0x10000 = "flash"
}

function Get-Ports {
    return @([System.IO.Ports.SerialPort]::GetPortNames() | Sort-Object -Unique)
}

function Write-Timed([string]$message) {
    Write-Host ("{0}  {1}" -f (Get-Date -Format "HH:mm:ss.fff"), $message)
}

function Format-ResetCause([uint32]$cause) {
    if ($cause -eq 0) {
        # The nRF sets no flag for a power-on reset
        return "none, so power-on"
    }
    $names = foreach ($flag in $ResetCauses.Keys) {
        if ($cause -band $flag) { $ResetCauses[$flag] }
    }
    return ($names -join ", ")
}

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
    Write-Timed "$port appeared"

    $serial = New-Object System.IO.Ports.SerialPort $port, $BaudRate, ([System.IO.Ports.Parity]::None), 8, ([System.IO.Ports.StopBits]::One)
    $serial.DtrEnable = -not $NoDtr
    $serial.RtsEnable = [bool]$Rts
    # Byte for byte, so the WP's binary pairing requests can't upset the text search
    $serial.Encoding = [System.Text.Encoding]::GetEncoding(28591)

    # The port can be listed a little before it can be opened, so keep trying
    $timer = [System.Diagnostics.Stopwatch]::StartNew()
    $attempts = 0
    $opened = $false
    while (-not $opened -and $timer.ElapsedMilliseconds -lt $OpenTimeoutMs) {
        $attempts++
        try {
            $serial.Open()
            $opened = $true
        } catch {
            Start-Sleep -Milliseconds $PollMs
        }
    }

    if ($opened) {
        # 0x55 alternates its bits, so unlike 0x00 it can't be mistaken for a break. Repeated in case the first is lost
        # while the line settles after the bridge powers up; the WP only records the first that arrives.
        $serial.Write([byte[]]@(0x55), 0, 1)
        Write-Timed ("{0} opened {1} ms after it appeared ({2} attempts), first byte sent" -f $port, $timer.ElapsedMilliseconds, $attempts)
        for ($i = 0; $i -lt 10; $i++) {
            Start-Sleep -Milliseconds 50
            $serial.Write([byte[]]@(0x55), 0, 1)
        }

        $received = ""
        $reply = $null
        $wait = [System.Diagnostics.Stopwatch]::StartNew()
        while (-not $reply -and $wait.ElapsedMilliseconds -lt $ReplyTimeoutMs) {
            Start-Sleep -Milliseconds 20
            $received += $serial.ReadExisting()
            $reply = [regex]::Match($received, "WPTEST first_byte_ms=(\d+) value=0x([0-9A-Fa-f]{2}) reset_cause=0x([0-9A-Fa-f]{8})")
            if (-not $reply.Success) {
                $reply = $null
            }
        }

        if ($reply) {
            $cause = [Convert]::ToUInt32($reply.Groups[3].Value, 16)
            Write-Timed ("WP: first byte 0x{0} arrived {1} ms after it started, reset cause 0x{2} ({3})" -f
                         $reply.Groups[2].Value, $reply.Groups[1].Value, $reply.Groups[3].Value, (Format-ResetCause $cause))
        } elseif ($received.Length -gt 0) {
            Write-Timed ("No WPTEST reply within {0} ms, but {1} bytes were received. Is the WP running the test build?" -f
                         $ReplyTimeoutMs, $received.Length)
        } else {
            Write-Timed ("Nothing received from the WP within {0} ms" -f $ReplyTimeoutMs)
        }
        $serial.Close()
    } else {
        Write-Timed ("{0} couldn't be opened within {1} ms, is another program using it?" -f $port, $OpenTimeoutMs)
    }
    $serial.Dispose()

    Write-Timed "Unplug the device to measure again"
    while ((Get-Ports) -contains $port) {
        Start-Sleep -Milliseconds 100
    }
    $known = Get-Ports
    Write-Timed "$port removed"
}
