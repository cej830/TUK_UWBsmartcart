package com.example.tcp_2.service

import java.io.*
import java.net.Socket



class TcpClient(
    private val ip: String,
    private val port: Int,
    private val onMessage: (String) -> Unit,
    private val onConnected: () -> Unit,
    private val onError: (String) -> Unit //

) : Thread() {

    private lateinit var socket: Socket
    private lateinit var writer: BufferedWriter
    private lateinit var reader: BufferedReader
    @Volatile var running = true

    override fun run() {
        try {
            socket = Socket(ip, port)
            writer = BufferedWriter(OutputStreamWriter(socket.getOutputStream()))
            reader = BufferedReader(InputStreamReader(socket.getInputStream()))
            onConnected()

            while (running) {
                // shutdownInput()이 호출되면 readLine()은 null을 반환하고 루프가 종료됩니다.
                val line = reader.readLine() ?: break
                onMessage(line)
            }

        } catch (e: Exception) {
            if (running) {

                onError("연결 실패: ${e.message}")
                e.printStackTrace()
            }
        } finally {
            try {
                if (::reader.isInitialized) reader.close()
                if (::writer.isInitialized) writer.close()
                if (::socket.isInitialized) socket.close()
            } catch (e: IOException) {
                e.printStackTrace()
            }
        }
    }


    fun send(cmd: String) {

        Thread {
            try {
                if (::writer.isInitialized) {
                    writer.write(cmd + "\n")
                    writer.flush()
                }
            } catch (e: Exception) {
                e.printStackTrace()
            }
        }.start()
    }


    fun stopClient() {
        running = false
        // 소켓을 직접 닫는 대신, 입력 스트림을 닫아 readLine()의 대기를 풀어줍니다.
        if (::socket.isInitialized && !socket.isClosed) {
            try {
                socket.shutdownInput()
            } catch (e: IOException) {
                e.printStackTrace()
            }
        }
    }

}