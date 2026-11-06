package com.example.tcp_2.ui

import android.content.ComponentName
import android.content.Context
import android.content.Intent
import android.content.ServiceConnection
import android.os.Bundle
import android.os.IBinder
import android.view.View
import android.widget.Button
import android.widget.TextView
import android.widget.Toast
import androidx.activity.enableEdgeToEdge
import androidx.appcompat.app.AppCompatActivity
import com.example.tcp_2.R
import com.example.tcp_2.service.MyTcpService




class MainActivity : AppCompatActivity() {


    private var myTcpService: MyTcpService? = null //서비스(MyTcpService)를 담아둘 변수
    private var isServiceBound = false  //isServiceBound는 서비스에 연결(bind)되어 있는지를 기록


    private lateinit var modeTextView: TextView //화면에 있는 글씨(TextView)와 버튼들을 코드에서 사용할 준비.
    private lateinit var distanceTextView: TextView

    private lateinit var connectionButton: Button
    private lateinit var disCButton: Button
    private lateinit var toggleButton: Button
    private lateinit var resumeButton: Button

    private lateinit var warningsmessage:TextView




    private val serviceConnection = object : ServiceConnection {  //서비스에 연결하거나 끊겼을 때 실행되는 코드
        override fun onServiceConnected(name: ComponentName?, service: IBinder?) {
            val binder = service as MyTcpService.LocalBinder
            myTcpService = binder.getService()
            isServiceBound = true
            observeService()
        }

        override fun onServiceDisconnected(name: ComponentName?) {
            isServiceBound = false
            myTcpService = null
        }
    }


    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        enableEdgeToEdge()
        setContentView(R.layout.activity_main)

        //UI 요소
        modeTextView = findViewById(R.id.statusTextView)
        distanceTextView =findViewById(R.id.distanceTextView)

        connectionButton = findViewById(R.id.connectButton)
        disCButton = findViewById(R.id.disconnectButton)
        toggleButton = findViewById(R.id.toggleButton)

        warningsmessage = findViewById(R.id.warrningMessage)
        resumeButton = findViewById(R.id.RESUME)


        // 초기 상태
        disCButton.isEnabled = false
        toggleButton.isEnabled = false
        resumeButton.isEnabled = false

        warningsmessage.visibility = View.GONE


        connectionButton.setOnClickListener {

            // 연결 버튼은 서비스를 시작하는 역할만 담당
            val serviceIntent = Intent(this, MyTcpService::class.java)
            startService(serviceIntent)

            // 2. 새로 시작된 서비스에 바인딩하여 통신을 시작합니다.
            bindService(serviceIntent, serviceConnection, Context.BIND_AUTO_CREATE)

            // 연결 버튼을 누르면:
            // 서비스(MyTcpService)를 시작하고
            // 서비스에 연결(bind)해서 myTcpService를 가져옴
        }

        disCButton.setOnClickListener {

            // 연결 해제 버튼은 서비스를 완전히 종료
            val serviceIntent = Intent(this, MyTcpService::class.java)

            if (isServiceBound) {

                unbindService(serviceConnection)
                isServiceBound = false

            }

            stopService(serviceIntent) // 서비스 종료 요청
            Toast.makeText(this, "연결 해제", Toast.LENGTH_SHORT).show()
            updateUi(false) // 즉시 UI 업데이트
        }

        toggleButton.setOnClickListener {
            if (!isServiceBound) {
                Toast.makeText(this, "먼저 연결하세요.", Toast.LENGTH_SHORT).show()
                return@setOnClickListener
            }

            if (toggleButton.text == "자동 추적 OFF") {
                myTcpService?.sendMessage("STOP")

            } else {
                myTcpService?.sendMessage("START")

            }
        }

        resumeButton.setOnClickListener{
            if(!isServiceBound){
                Toast.makeText(this, "먼저 연결하세요.", Toast.LENGTH_SHORT).show()
                return@setOnClickListener
            }

            else {
                myTcpService?.sendMessage("RESUME")
                Toast.makeText(this, "대기 모드를 해제합니다.", Toast.LENGTH_SHORT).show()

            }            }


    }

    private fun observeService() {
        myTcpService?.connectionStatus?.observe(this) { connected ->
            updateUi(connected)
        }

        myTcpService?.receivedMessage?.observe(this) { message ->

            try {
                // message 예시: "DIST:1.23 MODE:TRACKING"
                val parts = message.split(" ") // 공백으로 분리 -> ["DIST:1.23", "MODE:TRACKING"]
                val dist = parts[0].substringAfter("DIST:")
                val mode = parts[1].substringAfter("MODE:")

                // 각각의 TextView에 텍스트 설정
                distanceTextView.text = dist + " m"
                modeTextView.text = mode

                //MODE 값에 따라 버튼 상태 변경
                // 2. 파싱한 'MODE' 값에 따라 UI 변경
                when (mode) {
                    "WAITING_FOR_RESUME" -> {

                        // 'RESUME'이 필요한 상태이므로 RESUME 버튼 활성화
                        resumeButton.isEnabled = true
                        toggleButton.isEnabled = false
                        disCButton.isEnabled = true
                        warningsmessage.visibility = View.VISIBLE

                    }

                    "TRACKING", "PROXIMITY_STOP",  -> {

                        resumeButton.isEnabled = false
                        toggleButton.isEnabled = true
                        disCButton.isEnabled = true
                        toggleButton.text = "자동 추적 OFF"

                        warningsmessage.visibility = View.GONE

                    }


                    else -> { // "WAITING", "TRACKING_FAIL" 등 모든 비추적 모드

                        resumeButton.isEnabled = false
                        toggleButton.isEnabled = true
                        disCButton.isEnabled = true

                        toggleButton.text = "자동 추적 ON"

                        warningsmessage.visibility = View.GONE
                    }
                }
            } catch (e: Exception) {
                // 메시지 형식이 잘못되었을 경우를 대비
                distanceTextView.text = "---"
                modeTextView.text = "Error"

            }


        }

    }


    private fun updateUi(connected: Boolean) {

        // 연결/해제 시의 버튼 활성화만 담당

        disCButton.isEnabled = connected
        toggleButton.isEnabled = connected

        if (!connected) {

        }
            distanceTextView.text = "---"
            modeTextView.text = "연결 끊김"
            toggleButton.text = "자동 추적 ON"
            resumeButton.isEnabled = false
            warningsmessage.visibility = View.GONE
    }


    override fun onStart() {
        super.onStart()
        // Activity가 화면에 보일 때마다 서비스에 바인딩 시도
        // 서비스가 실행 중이 아니면 아무 일도 일어나지 않음
        val serviceIntent = Intent(this, MyTcpService::class.java)
        bindService(serviceIntent, serviceConnection, Context.BIND_AUTO_CREATE)
    }

    override fun onStop() {
        super.onStop()
        // Activity가 화면에서 사라지면 바인딩 해제
        if (isServiceBound) {
            unbindService(serviceConnection)
            isServiceBound = false
        }
    }
}


