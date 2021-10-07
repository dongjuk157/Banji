<template>
  <div v-if="checkRobot">
    <h1 class="loadingMsg">
      로봇<br>찾는중...
    </h1>
    <div class="loadingSpinner">
      <b-spinner></b-spinner>
    </div>
  </div>
  <div v-else>
    <div class="status">
      <h2>로봇 상태 정보</h2>
      <b-list-group>
        <b-list-group-item v-for="(data, name) in status" :key="name">
          {{ statusName[name] }} : {{ status[name] }}
        </b-list-group-item>
        <b-btn @click="updateRobotStatus">Update</b-btn>
      </b-list-group>
      <hr>

      <h2>환경 정보 </h2>
      <b-list-group>
        <b-list-group-item v-for="(data, name) in envir" :key="name">
          {{ envirName[name] }} : {{ data }}
        </b-list-group-item>
        <b-btn @click="updateEnvironment">Update</b-btn>
      </b-list-group>
    </div>
  </div>
</template>

<script>
export default {
  name: 'RobotStatus',
  data() {
    return {
      checkRobot: true,
      // 추후에 vuex state에 해당 로봇 상태 데이터를 저장할 예정.
      status: {
        battery: null,
        power: null,
        // '배터리 상태 :': '97%',
        // '전력 공급 상태 : ': 4,
        // 더 추가할 것이 있는지
      },
      envir: {
        time: '',
        temperature: '',
        weather: '',
      },
      envirName: {
        time: '시간',
        temperature: '온도',
        weather: '날씨',
      },
      statusName: {
        battery: '배터리',
        power: '전력 공급 상태',
      },
    };
  },
  mounted() {
    this.$socket.on('front_environment_back', (message) => {
      const minute = (message.minute < 10) ? `0${String(message.minute)}` : String(message.minute);
      this.envir.time = `${message.month}월 ${message.day}일 ${message.hour}:${minute}`;
      this.envir.weather = message.weather;
      this.envir.temperature = `${message.temperature} ℃`;
      this.checkRobot = false;
    });
    this.$socket.on('front_robotStatus_back', (message) => {
      this.status.battery = message.battery;
      this.status.power = message.power;
      this.checkRobot = false;
    });
    this.updateEnvironment();
    this.updateRobotStatus();
  },
  // 찾는 거를 왜하냐? 1. 배터리 나갔을 수도 있는데... 근데 연결은 되지않나?
  // 이 부분 한번 같이 말해보기 ㅇㅇ
  // 연결 안될수도 있지 않나요? 제일 처음 로봇이랑 연결이 되었는지는 파악해야할거같아서요
  methods: {
    // changeRobotStatus() {
    //   this.checkRobot = false;
    // },
    updateEnvironment() {
      const data = { key: 1 };
      this.$socket.emit('back_environment_front', data);
    },
    updateRobotStatus() {
      const data = { key: 2 };
      this.$socket.emit('back_robotStatus_front', data);
    },
  },
  created() {
  },
};
</script>

<style scoped>
  .status {
    padding: 1rem;
    animation: fadein 2.5s;
    -webkit-animation: fadein 2.5s; /* Safari and Chrome */
  }
  .loadingMsg {
    position: absolute;
    top: 40%;
    left: 50%;
    transform: translate(-50%, -50%);
  }
  .loadingSpinner {
    position: absolute;
    top: 60%;
    left: 50%;
    transform: translate(-50%, -50%);
  }

  @keyframes fadein {
      from {
          opacity: 0;
      }
      to {
          opacity: 1;
      }
  }
  @-webkit-keyframes fadein { /* Safari and Chrome */
      from {
          opacity: 0;
      }
      to {
          opacity: 1;
      }
  }

</style>
