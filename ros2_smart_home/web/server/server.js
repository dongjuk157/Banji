
// Websocket 서버 구동을 위한 서버 코드입니다.

// 노드 로직 순서

const path = require('path');
const express = require('express');

// client 경로의 폴더를 지정해줍니다.
// const publicPath = path.join(__dirname, "/../client");
var app = express();

const picPath = path.join(__dirname, "/../client");

// app.use(express.static(publicPath));

// 로직 1. WebSocket 서버, WebClient 통신 규약 정의
const server = require('http').createServer(app);
const io = require('socket.io')(server, { 
    cors: { origin: "*" },
    transports: ['websocket', 'polling'],
})

var fs = require('fs'); // required for file serving

// 로직 2. 포트번호 지정
const port = process.env.port || 12001

server.listen(port, () => {
    console.log(`listening on *:${port}`);
});

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
        // console.log('로봇 정보 줘')
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
        // message: one of directions 'go', 'back', 'left', 'right'
        console.log(message)
        socket.to(roomName).emit('robot_move_back', message)
    })
    
    
    // 로직 4 로봇의 메시지 수신
    socket.on('back_environment_robot', (message) => {
        // console.log('환경 정보 줄게')
        socket.to(roomName).emit('front_environment_back', message);
    });
    socket.on('back_robotStatus_robot', (message) => {
        // console.log('로봇 정보 줄게')
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




    socket.on('disconnect', () => {
        console.log('disconnected from server');
    });

    // 전달받은 이미지를 jpg 파일로 저장
    socket.on('streaming', (message) => {
        socket.to(roomName).emit('sendStreaming', message);
        // console.log(message);
        buffer = Buffer.from(message, "base64");
        fs.writeFileSync(path.join(picPath, "/../client/cam.jpg"), buffer);
    });

})