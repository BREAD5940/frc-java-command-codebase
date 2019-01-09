./gradlew build

if [ "$(networksetup -getairportnetwork en0 | cut -c 24-)" == *5940* ] then
  ./gradlew deploy
else
  echo "Not connected to the 5940 network!"
fi
