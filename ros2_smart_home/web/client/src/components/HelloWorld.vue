<template>
  <v-row class="pa-3">
    <v-col cols="6">
      <!-- 로직 1. textarea 생성 -->
      <p>Security Status</p>
      <v-textarea id="tSafetyStatus" rows="2"></v-textarea>
    </v-col>
    <v-col cols="6">
      <!-- 로직 2. 침입자 인지 시점 이미지 뷰어 생성 -->
      <v-img :src="imageSrc" width="480" @load="imageLoad" @error="imageError"></v-img>
    </v-col>
    <v-col cols="6">
      <!-- 로직 3. 순찰 모드 스위치 버튼 생성 -->
      <p>Patrol On/Off Status</p>
      <v-textarea id="tPatrolStatus" rows="2">{{}}</v-textarea>
      <v-btn id="A1-btn" @click="btn_patrol_on">On</v-btn>
      <v-btn id="A2-btn" @click="btn_patrol_off">Off</v-btn>
    </v-col>

    <v-col cols="6">
      <!-- 로직 4. 수동 컨트롤 버튼 생성 -->
      <p>Manual Controller</p>
      <v-btn id="A1-btn" @click="btn_turn_left">Turn Left</v-btn>
      <v-btn id="A2-btn" @click="btn_go_straight">Go Straight</v-btn>
      <v-btn id="A2-btn" @click="btn_turn_right">Turn Right</v-btn>
    </v-col>
    {{ temp }}
    <v-col cols="6">
      <p>TimeInfo</p>
      <textarea id="tAreaTime" rows="4"></textarea>
      <p>WeatherInfo</p>
      <textarea id="tAreaWeather" rows="4"></textarea>
      <p>TemperatureInfo</p>
      <textarea id="tAreaTemp" rows="4"></textarea>
      <v-btn id="A1-btn" @click="status_click_on">상태 정보 얻기 </v-btn>
    </v-col>
    <v-col cols="6">
      <p>가전 제품 켜기 / 끄기</p>
      <v-btn id="btn" @click="airPurifier_btn_click(1)">On / OFF</v-btn>
    </v-col>
  </v-row>
</template>

<script>
export default {
  data() {
    return {
      time: new Date().getTime(),
      temp: 0,
    };
  },
  mounted() {
    // this.$socket.on("chat", (data) => {});

    this.$socket.on('sendTimeToWeb', (message) => {
      console.log('sendTimeToWeb', message);
      document.querySelector('#tAreaTime').value = message;
    });

    this.$socket.on('sendWeatherToWeb', (message) => {
      console.log('sendWeatherToWeb', message);
      document.querySelector('#tAreaWeather').value = message;
    });

    this.$socket.on('sendTemperatureToWeb', (message) => {
      console.log('sendTemperatureToWeb', message);
      this.temp = message;
    });

    this.$socket.on('sendAirConditionerToWeb', (message) => {
      console.log('sendAirConditionerToWeb', message);
      document.querySelector('#tAreaAircon').value = message;
    });
  },
  socket: {
    disconnect() {
      console.log('disconnected form server_client.');
    },
    // 로직 1. 서버에서 온 메시지를 웹페이지에 전달
    sendTimeToWeb(message) {
      console.log('sendTimeToWeb', message);
      document.querySelector('#tAreaTime').value = message;
    },
    sendWeatherToWeb(message) {
      console.log('sendWeatherToWeb', message);
      document.querySelector('#tAreaWeather').value = message;
    },
    sendTemperatureToWeb(message) {
      console.log('sendTemperatureToWeb', message);
      document.querySelector('#tAreaTemp').value = message;
    },
    sendAirConditionerToWeb(message) {
      console.log('sendAirConditionerToWeb', message);
      document.querySelector('#tAreaAircon').value = message;
    },

    // 로직 1. 서버에서 온 메시지를 웹페이지에 전달
    sendSafetyStatus(message) {
      console.log('sendSafetyStatus', message);
      document.querySelector('#tSafetyStatus').value = message;
    },
    sendPatrolStatus(message) {
      console.log('sendPatrolStatus', message);
      document.querySelector('#tPatrolStatus').value = message;
    },
  },
  methods: {
    imageLoad() {
      setTimeout(() => {
        this.time = new Date().getTime();
        // console.log(this.time)
      }, 50);
    },
    imageError() {
      setTimeout(() => {
        this.time = new Date().getTime();
        // console.log(this.time)
      }, 50);
    },
    btn_patrol_on() {
      console.log('btn_patrol_on');
      const data = 1;
      this.$socket.emit('PatrolOnToServer', data);
    },
    btn_patrol_off() {
      console.log('btn_patrol_off');
      const data = 0;
      this.$socket.emit('PatrolOffToServer', data);
    },
    btn_turn_left() {
      console.log('btn_left');
      const data = 1;
      this.$socket.emit('turnleftToServer', data);
    },
    btn_go_straight() {
      console.log('btn_go_straight');
      const data = 2;
      this.$socket.emit('gostraightToServer', data);
    },
    btn_turn_right() {
      console.log('btn_turn_right');
      const data = 3;
      this.$socket.emit('turnrightToServer', data);
    },
    status_click_on() {
      console.log('status_click_on');
      const data = { key: 1 };
      this.$socket.emit('sendStateRefreshToServer', data);
    },
    btn_click_on() {
      console.log('btn_click_on');
      const data = { key: 1 };
      this.$socket.emit('sendAirConOnToServer', data);
    },
    btn_click_off() {
      console.log('btn_click_off');
      const data = { key: 2 };
      this.$socket.emit('sendAirConOffToServer', data);
    },
    light_btn_click(num) {
      console.log('light_btn_click', num);
      const data = { key: 3 };
      this.$socket.emit('sendLightOnToServer', data);
    },
    airPurifier_btn_click(num) {
      console.log('airPurifier_btn_click', num);
      const data = { key: 4 };
      this.$socket.emit('sendAirPurifierOnToServer', data);
    },
    blind_btn_click(num) {
      console.log('blind_btn_click', num);
      const data = { key: 5 };
      this.$socket.emit('sendBlindOnToServer', data);
    },
  },
  computed: {
    imageSrc() {
      const fileName = 'cam.jpg';
      const query = '?t=';
      // console.log(fileName + query + this.time)
      return fileName + query + this.time;
    },
  },
};
</script>

<style></style>
