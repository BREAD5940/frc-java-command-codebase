./gradlew build -x check

$name = (Get-NetConnectionProfile -InterfaceAlias "Wi-Fi").name
if ( $name -match "5940" ) {
  # Connected to some sort of 5940 network
  ./gradlew deploy
} else {
  Write-Output "Not connected to the 5940 network!"
}