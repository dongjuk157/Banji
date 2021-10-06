
// Websocket 서버 구동을 위한 서버 코드입니다.

// 노드 로직 순서

const path = require('path');
const express = require('express');

// client 경로의 폴더를 지정해줍니다.
// const publicPath = path.join(__dirname, "/../client");
var app = express();

const picPath = path.join(__dirname, "/./images/");

// app.use(express.static(publicPath));

// 로직 1. WebSocket 서버, WebClient 통신 규약 정의
const server = require('http').createServer(app);
// const server = require('https').createServer(app);
let privateKey = null
let certificate = null
let ca = null
try {
    privateKey = fs.readFileSync('/etc/letsencrypt/live/j5b301.p.ssafy.io/privkey.pem').toString();
    certificate = fs.readFileSync('/etc/letsencrypt/live/j5b301.p.ssafy.io/cert.pem').toString();
    ca = fs.readFileSync('/etc/letsencrypt/live/j5b301.p.ssafy.io/fullchain.pem').toString();
} catch {
    console.log('No ssl file')
}

const io = require('socket.io')(server, { 
    secure: true,
    key: privateKey,
    cert: certificate,
    ca: ca,
    cors: { origin: "*" },
    transports: ['websocket', 'polling'],
    
})

var fs = require('fs'); // required for file serving

// 로직 2. 포트번호 지정
const port = process.env.port || 12001

server.listen(port, () => {
    console.log(`listening on *:${port}`);
});

// server
app.use('/images', express.static('images'));


// socket.io
const roomName = 'team';

io.on('connection', socket => {
    socket.join(roomName);
    // 로직 3. 사용자(frontend)의 메시지 수신시 WebClient로 메시지 전달
    // message = [month, day, hour, minute]
    socket.on('back_environment_front', (message) => {
        // console.log('환경 정보 줘')
        socket.to(roomName).emit('robot_environment_back', message);
    });
    socket.on('back_robotStatus_front', (message) => {
        console.log('로봇 정보 줘')
        socket.to(roomName).emit('robot_robotStatus_back', message);
    });
    socket.on('back_control_front', (message) => {
        // console.log('가전 제어 해')
        socket.to(roomName).emit('robot_control_back', message)
    });
    socket.on('back_loadmap_front', (message) => {
        // console.log('지도 줘')
        socket.to(roomName).emit('robot_loadmap_back', message)
    });
    socket.on('back_move_front', (message) => {
        // 좌표 눌렀을 때 자동 이동
        socket.to(roomName).emit('robot_move_back', message)
    });
    socket.on('back_movemanualy_front', (message) => {
        // message: one of directions 'go', 'back', 'left', 'right', and 'stop'
        // go, back: linear.x 값 조정
        // left, right: angular.z 값 조정
        // stop: x,z 모두 0으로
        // console.log(message)
        socket.to(roomName).emit('robot_movemanualy_back', message)
    });
    socket.on('back_screenshot_front', (message) => {
        console.log('스크린샷 줘')
        // console.log(message)
        socket.to(roomName).emit('robot_screenshot_back', message)
    });
    socket.on('back_robotview_front', (message) => {
        // console.log('실시간 영상 줘')
        // console.log('실시간 영상', message)
        socket.to(roomName).emit('robot_robotview_back', message)
    });
    socket.on('back_getImgList_front', (message) => {
        // 저장되어있는 파일 리스트 반환
        
        fs.readdir(path.join(picPath, message), (err, files) => {
            if (err) {
                return console.log("Unable to scan directory:", err)
            }
            console.log("In Dir,", files)
            socket.emit('front_getImgList_back', files);
            socket.to(roomName).emit('front_getImgList_back', files);
            // 이걸로 하면 roomName엔 보내는거같은데 프론트에서 못읽음. 다른 환경에서 아래 코드가 되면 위는 지워도됨
        })
    });
    socket.on('back_position_front', (message) => {
        // console.log('로봇 위치 줘')
        socket.to(roomName).emit('robot_position_back', message)
    });
    

// ###########################################################################################################################


    // 로직 4 로봇의 메시지 수신
    socket.on('back_environment_robot', (message) => {
        // console.log('환경 정보 줄게')
        socket.to(roomName).emit('front_environment_back', message);
    });
    socket.on('back_robotStatus_robot', (message) => {
        console.log('로봇 정보 줄게')
        socket.to(roomName).emit('front_robotStatus_back', message);
    });
    socket.on('back_control_robot', (message) => {
        // console.log('가전 제어 했어') 
        socket.to(roomName).emit('front_control_back', message);
    });
    socket.on('back_loadmap_robot', (message) => {
        // console.log('지도 줄게') 
        socket.to(roomName).emit('front_loadmap_back', message);
    });
    socket.on('back_screenshot_robot', (message) => {
        console.log('스크린샷 줄게')
        const curTime = new Date().toISOString();
        let date, time;
        [date, time] = curTime.split('T')
        date = date.split('-').join('')
        time = time.split(':')
        buffer = Buffer.from(message, "base64");
        fileName = `cam_${date}${time[0]}${time[1]}${time[2].slice(0,2)}.jpg`
        fs.writeFileSync(path.join(picPath, `screenshot/${fileName}`), buffer);
        // socket.to(roomName).emit('front_screenshot_back', message);
    })
    socket.on('back_robotview_robot', (message) => {    // 실시간 영상 부분
        console.log('실시간 영상 줄게')
        socket.to(roomName).emit('front_robotview_back', message);
    });
    socket.on('back_alert_robot', (message) => {
        // 전달받은 이미지를 jpg 파일로 저장
        // 현재 시각
        console.log(message)
        // try {
        const curTime = new Date().toISOString();
        let date, time;
        [date, time] = curTime.split('T')
        date = date.split('-').join('')
        time = time.split(':')
        // console.log(message);
        buffer = Buffer.from(message, "base64");
        fileName = `cam_${date}${time[0]}${time[1]}.jpg`
        fs.writeFileSync(path.join(picPath, `intruder/${fileName}`), buffer);    
        // } catch {
            
        // }
        // socket.emit('front_alert_back', 'intruder!!');
        socket.to(roomName).emit('front_alert_back', 'intruder!!');
    });
    socket.on('back_position_robot', (message) => {
        // console.log('로봇 위치 줄게')
        // console.log(message)
        socket.to(roomName).emit('front_position_back', message)
    });


    socket.on('disconnect', () => {
        console.log('disconnected from server');
    });

})