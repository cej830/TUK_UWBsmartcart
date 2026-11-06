# Android · 스마트 카트 제어 앱

[프로젝트 소개로 돌아가기](../README.md)

Raspberry Pi의 TCP 서버에 연결해 자동 추종을 제어하고, 카트와 사용자 사이의 거리 및 동작 상태를 표시하는 Android 앱 프로젝트이다. Android Studio에서 `android/` 폴더를 연다.

## 파일 구성

| 파일 | 패키지 | 역할 |
| --- | --- | --- |
| [MainActivity.kt](app/src/main/java/com/example/tcp_2/ui/MainActivity.kt) | `com.example.tcp_2.ui` | 버튼 이벤트, 서비스 바인딩, 거리·상태 표시 |
| [MyTcpService.kt](app/src/main/java/com/example/tcp_2/service/MyTcpService.kt) | `com.example.tcp_2.service` | 포그라운드 서비스, 연결 관리, LiveData 갱신 |
| [TcpClient.kt](app/src/main/java/com/example/tcp_2/service/TcpClient.kt) | `com.example.tcp_2.service` | 소켓 연결, 줄 단위 상태 수신, 명령 전송 |

## 프로젝트 구성과 실행 준비

앱 소스, 화면 리소스, Manifest, Gradle 설정을 하나의 Android Studio 프로젝트로 구성했다.

| 경로 | 내용 |
| --- | --- |
| `settings.gradle.kts` | 프로젝트 및 모듈 설정 |
| `app/build.gradle.kts` | SDK, 앱 빌드 및 의존성 설정 |
| `gradle/libs.versions.toml` | 주요 플러그인·라이브러리 버전 |
| `app/src/main/AndroidManifest.xml` | 권한, Activity, Service 등록 |
| `app/src/main/java/` | 실제 빌드에 사용하는 Kotlin 소스 |
| `app/src/main/res/` | 레이아웃, 문자열, 테마, 아이콘 |

현재 빌드 설정은 `compileSdk = 35`, `minSdk = 24`, `targetSdk = 34`이다. Android Studio에서 `android/`를 열고 필요한 SDK 및 Gradle 의존성을 준비한 뒤 동기화한다. 서버 IP를 변경하고 앱을 빌드·설치한다. SDK 경로 등 개발 PC별 설정은 해당 환경에서 구성한다.

소스가 참조하는 리소스는 다음과 같다. ID는 코드에 적힌 철자와 대소문자를 그대로 사용한다.

| 종류 | 이름 |
| --- | --- |
| 레이아웃 | `activity_main` |
| TextView | `statusTextView`, `distanceTextView`, `warrningMessage` |
| Button | `connectButton`, `disconnectButton`, `toggleButton`, `RESUME` |
| Drawable | `ic_notification` |

## 서버 연결 설정

`app/src/main/java/com/example/tcp_2/service/MyTcpService.kt`의 `startClient()`에 서버 주소가 지정되어 있다.

```kotlin
val ip = "192.168.5.70"
val port = 5000
```

IP는 개발 당시 설정값이므로 사용할 Raspberry Pi 주소로 변경한다. 포트는 [Raspberry Pi 서버](../controller/README.md)의 `PORT` 값과 일치해야 한다.

## 사용 흐름

1. Raspberry Pi 제어 프로그램을 실행한다.
2. 스마트폰과 Raspberry Pi를 서로 접근 가능한 네트워크에 연결한다.
3. 앱의 연결 버튼으로 TCP 서비스를 시작한다.
4. 추종 버튼으로 `START` 또는 `STOP`을 전송한다.
5. `WAITING_FOR_RESUME` 상태에서는 대기 해제 버튼으로 `RESUME`을 전송한다. 이후 추종을 다시 시작해야 한다.
6. 연결 해제 버튼으로 서비스를 종료한다.

서버 메시지 `DIST:1.23 MODE:TRACKING`을 공백으로 나눠 거리와 상태에 표시한다. `TcpClient`는 `readLine()`으로 수신하므로 서버 메시지에는 줄바꿈이 필요하다. 전송 명령에도 `\n`을 붙인다.

## 화면 상태

| 서버 상태 | 화면 동작 |
| --- | --- |
| `WAITING_FOR_RESUME` | 경고 표시, 대기 해제 버튼 활성화, 추종 버튼 비활성화 |
| `TRACKING`, `PROXIMITY_STOP` | 추종 중 상태로 표시, 추종 정지 버튼 사용 |
| 그 외 상태 | 추종 시작 버튼 표시, 대기 해제 버튼 비활성화 |
