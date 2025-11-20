package com.example.tcp_2.service

import android.app.NotificationChannel
import android.app.NotificationManager
import android.app.Service
import android.content.Intent
import android.os.Binder
import android.os.Build
import android.os.IBinder
import androidx.core.app.NotificationCompat
import androidx.lifecycle.MutableLiveData
import com.example.tcp_2.R

class MyTcpService : Service() {

    private lateinit var client: TcpClient  //TcpClient는 실제 TCP 연결을 담당하는 별도 스레드.
    private var isConnected = false  //TCP 연결 상태 확인 변수
    private var tracking = false  //서비스가 기억하는 상태변수

    val connectionStatus = MutableLiveData<Boolean>()  //LiveData로 MainActivity가 서비스의 상태를 관찰 가능.
    val receivedMessage = MutableLiveData<String>()  // 받은 메세지, LiveData는 서비스가 상태를 바꾸면, 화면(MainActivity)에 자동으로 알립니다
    val trackingStatus = MutableLiveData<Boolean>()

    private val binder = LocalBinder()  // 화면과 서비스를 연결해주는 Binder라는 장치

    inner class LocalBinder : Binder() {                       //Activity가 서비스에 바인딩해서 getService()로 참조 가능.
        fun getService(): MyTcpService = this@MyTcpService      //화면에서 서비스에 접근할 수 있게, 이 서비스를 반환 클래스
    }

    override fun onBind(intent: Intent?): IBinder = binder  //생명주기  , 화면에서 bindService() 호출 시 이 메서드가 실행되고 binder를 반환.

    override fun onStartCommand(intent: Intent?, flags: Int, startId: Int): Int { // 서비스가 시작될 때 실행 알림(Notification)과 함께 실행되고, TCP 연결을 시작

        startForegroundService()  // 포그라운드로 전환
        startClient()        // TCP 연결 시작.
        return START_STICKY  //꺼졌다가도 자동으로 다시 실행.
    }

    private fun startClient() {

        if (isConnected) return

        val ip = "192.168.5.70"
        val port = 5000

        client = TcpClient(ip, port,
            onMessage = { message ->
                receivedMessage.postValue(message)   //메시지를 받으면 → 화면에 알려줌
            },
            onConnected = {
                isConnected = true

                connectionStatus.postValue(true)   //연결되면 → 화면에 알려줌

            },
            onError = { error ->
                isConnected = false
                connectionStatus.postValue(false)  //에러나면 → 화면에 알려줌
            }
        )   // TCP 연결 객체(TcpClient)를 만들기

        client.start()  //실행
    }

    fun sendMessage(cmd: String) {   //Activity에서 서비스에 메시지를 보내면 → 서비스가 TCP를 통해 전송.
        if (isConnected) {

            if (cmd == "START") {
                tracking = true
            } else if (cmd == "STOP") {
                tracking = false
            }

            // LiveData에 변경된 상태를 알려줌
            trackingStatus.postValue(tracking)
            client.send(cmd)

        }
    }


    /*fun disconnect() {  //연결을 끊고 싶을 때 실행
        if (::client.isInitialized) {
            client.stopClient()
            isConnected = false
            connectionStatus.postValue(false)
        }
    }*/


    private fun startForegroundService() {  //안드로이드 8.0 이상에서는 포그라운드 서비스에 알림 채널이 필요
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.O) {

            val channel = NotificationChannel(
                "tcp_channel",
                "TCP 연결",
                NotificationManager.IMPORTANCE_LOW
            )
            val manager = getSystemService(NotificationManager::class.java)
            manager.createNotificationChannel(channel)
        }

        val notification = NotificationCompat.Builder(this, "tcp_channel")
            .setContentTitle("TCP 서비스 실행중")
            .setContentText("서버에 연결중…")
            .setSmallIcon(R.drawable.ic_notification)
            .build()


        startForeground(1, notification)  //알림을 띄우고 서비스를 포그라운드로 실행
    }



    override fun onDestroy() {  //서비스가 종료될 때, TCP 연결을 정리
        super.onDestroy()
        if (::client.isInitialized) {
            client.stopClient()
        }

        // 2. 서비스 내부 상태 변수들 초기화
        isConnected = false
        tracking = false

        // 3. LiveData를 통해 UI에 상태 변경 알림 (중요)
        connectionStatus.postValue(false)
        trackingStatus.postValue(false)
    }
}



