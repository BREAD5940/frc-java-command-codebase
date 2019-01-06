$name = (Get-NetConnectionProfile -InterfaceAlias "Wi-Fi").name
if ( $name -match "5940" ) {
  # Connected to some sort of 5940 network
  (gradlew build) -and (gradlew deploy)
} else {
  Write-Output "Not connected toa 5940 network!"
}
