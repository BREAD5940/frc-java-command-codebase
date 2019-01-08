$name = (Get-NetConnectionProfile -InterfaceAlias "Wi-Fi 4").name
if ( $name -match "5940" ) {
  # Connected to some sort of 5940 network
  ./buildDeploy.bat
} else {
  Write-Output "Not connected toa 5940 network!"
}
