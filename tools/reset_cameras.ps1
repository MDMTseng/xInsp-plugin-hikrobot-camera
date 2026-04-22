# reset_cameras.ps1 — disable + re-enable all HikRobot USB cameras.
#
# Use when MVS starts returning MV_E_USB_WRITE (0x80000301) on every
# feature set, the cameras enumerate but misbehave, or a previous
# process crashed mid-stream and left the camera firmware stuck.
#
# Requires admin rights the first time (UAC prompt). Subsequent
# invocations in the same elevated shell run without prompting.
#
# Usage:
#     powershell -ExecutionPolicy Bypass -File reset_cameras.ps1
#
# Returns exit code 0 on success, nonzero if no cameras were found
# or the PnP API refused the operation.

$ErrorActionPreference = 'Stop'

# HikRobot uses USB VID 2BDF. We target the "USB Composite Device"
# entries (the parent nodes) — disabling those causes Windows to tear
# down and re-enumerate all child interfaces, which resets the camera
# firmware's USB state machine cleanly.
$cams = Get-PnpDevice -PresentOnly |
    Where-Object { $_.InstanceId -like 'USB\VID_2BDF*' -and
                   $_.InstanceId -notlike '*MI_*' }

if (-not $cams) {
    Write-Host 'No HikRobot cameras found (VID 2BDF).'
    exit 1
}

foreach ($c in $cams) {
    Write-Host ('→ disabling {0}  ({1})' -f $c.FriendlyName, $c.InstanceId)
    Disable-PnpDevice -InstanceId $c.InstanceId -Confirm:$false
}

Start-Sleep -Seconds 2

foreach ($c in $cams) {
    Write-Host ('→ enabling  {0}  ({1})' -f $c.FriendlyName, $c.InstanceId)
    Enable-PnpDevice -InstanceId $c.InstanceId -Confirm:$false
}

# Give MVS's internal enumerator a moment to pick them back up.
Start-Sleep -Seconds 3

Write-Host ''
Write-Host 'done.' -ForegroundColor Green
exit 0
