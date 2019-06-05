@file:Suppress("MatchingDeclarationName")

package frc.ghrobotics.vision

import com.fazecast.jSerialComm.SerialPort
import com.fazecast.jSerialComm.SerialPortDataListener
import com.fazecast.jSerialComm.SerialPortEvent
import com.github.salomonbrys.kotson.fromJson
import com.google.gson.Gson
import com.google.gson.JsonObject
import com.google.gson.JsonParseException
//import edu.wpi.first.wpilibj.SerialPort
//import edu.wpi.first.wpilibj.SerialPort
//import edu.wpi.first.wpilibj.SerialPort
import edu.wpi.first.wpilibj.Timer
//import org.ghrobotics.frc2019.Constants
import org.ghrobotics.lib.mathematics.units.Time
import org.ghrobotics.lib.mathematics.units.second
import kotlin.concurrent.fixedRateTimer

object JeVoisManager {

    internal val kJevoisGson = Gson()

    private val connectedJeVoisCameras = mutableListOf<JeVois>()

    var isFrontJeVoisConnected = false
        private set

    var isBackJeVoisConnected = false
        private set

    init {

        fixedRateTimer(name = "JevoisManager", period = 3000L) {
            val currentTime = Timer.getFPGATimestamp().second

            var connect = ""
            connectedJeVoisCameras.forEach {
                connect += "name ${it.systemPortName} isFront ${it.isFront}"
            }

//            println("====> Updating Jevoises at ${currentTime.second}: current jevoises $connect")

            connectedJeVoisCameras.removeIf {
                it.update(currentTime)
                if (!it.isAlive) {
                    println("[JeVois Manager] Disconnected Camera: ${it.systemPortName}")
                    it.dispose()
                    return@removeIf true
                } else {
                    return@removeIf false
                }
            }

            val jeVoisSerialPorts = SerialPort.getCommPorts()
                .filter { it.descriptivePortName.contains("JeVois", true) }

            for (serialPort in jeVoisSerialPorts) {
                if (connectedJeVoisCameras.any { it.systemPortName.equals(serialPort.systemPortName, true) }) {
                    continue
                }
                println("========[JeVois Manager] Found new camera: ${serialPort.systemPortName} at baud ${serialPort.baudRate}")
                connectedJeVoisCameras.add(JeVois(serialPort))
            }

            isFrontJeVoisConnected = connectedJeVoisCameras.any { it.isFront == true }
            isBackJeVoisConnected = connectedJeVoisCameras.any { it.isFront == false }
        }

    }

    class JeVois(
        private val serialPort: SerialPort
    ) {

        var isFront: Boolean? = null
            private set

        val systemPortName: String = serialPort.systemPortName

        private var lastMessageReceived = 0.second
        private var wasUnplugged = false

        var isAlive = true
            private set

//        class TestListener : SerialPortDataListener {
//
//            override fun serialEvent(event: SerialPortEvent?) {
//                println("found data of type ${event!!.eventType}")
//            }
//
//            override fun getListeningEvents() = SerialPort.LISTENING_EVENT_DATA_AVAILABLE
//
//        }

        init {

            serialPort.openPort()
            serialPort.addDataListener(object : SerialPortDataListener {
                private var stringBuffer = StringBuilder(1024)

                override fun serialEvent(event: SerialPortEvent) {
                    try {

//                        println("serial event on jevois $systemPortName!")

                        if (event.eventType != SerialPort.LISTENING_EVENT_DATA_AVAILABLE)
                            return
                        val bytesAvailable = serialPort.bytesAvailable()
                        if (bytesAvailable < 0) {
                            wasUnplugged = true
                            return
                        }
                        val newData = ByteArray(bytesAvailable)
                        serialPort.readBytes(newData, newData.size.toLong())
                        for (newByte in newData) {
                            val newChar = newByte.toChar()
                            if (newChar != '\n') {
                                stringBuffer.append(newChar)
                                continue
                            }

                            processMessage(stringBuffer.toString())
                            stringBuffer.clear()
                        }



                    } catch (e: Throwable) {
                        println("[JeVois] ${e.localizedMessage}")
//                    e.printStackTrace()
                    }
                }

                override fun getListeningEvents() = SerialPort.LISTENING_EVENT_DATA_AVAILABLE
            })

//            serialPort.addDataListener(TestListener())

            lastMessageReceived = Timer.getFPGATimestamp().second
        }

        fun processMessage(message: String) {
            lastMessageReceived = Timer.getFPGATimestamp().second
            if (!message.startsWith('{')) return
            try {
                val jsonData = kJevoisGson.fromJson<JsonObject>(message)

                val isFront = jsonData["is_front"].asBoolean
                val timestamp = Timer.getFPGATimestamp() - jsonData["capture_ago"].asDouble
                val contours = jsonData["targets"].asJsonArray
                    .filterIsInstance<JsonObject>()

                this.isFront = isFront

//                println("processing data from the ${if(isFront) "front" else "back"} jevois, captured $timestamp, " +
//                        "with found contours $contours")

                VisionProcessing.processData(VisionData(isFront, timestamp, contours))
            } catch (e: JsonParseException) {
//            e.printStackTrace()
                println("[JeVois] Got Invalid Data: $message")
            }
        }

        fun update(currentTime: Time) {
            isAlive = !wasUnplugged && currentTime - lastMessageReceived <= TargetTracker.kVisionCameraTimeout
        }

        fun dispose() {
            serialPort.closePort()
        }

        private fun SerialPort.writeString(data: String) = writeBytes(data.toByteArray())
        private fun SerialPort.writeBytes(data: ByteArray) = writeBytes(data, data.size.toLong())

    }

}

data class VisionData(
    val isFront: Boolean,
    val timestamp: Double,
    val targets: List<JsonObject>
)