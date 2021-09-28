<template>
  <div v-if="checkRobot">
    <h1 style="margin-top: 50%;">로봇<br>찾는중...</h1>
    <div class="d-flex justify-content-center align-items-center" style="margin-top: 70px;">
      <b-spinner></b-spinner>
    </div>
  </div>
  <div v-else>
    <div class="status">
      <h2>로봇 상태 정보</h2>
      <b-list-group>
        <b-list-group-item v-for="(data, name) in status" :key="name">
          {{ name }} {{ data }}
        </b-list-group-item>
      </b-list-group>
      <hr>
      <h2>환경 정보 </h2>
      <b-list-group>
        <b-list-group-item v-for="(data, name) in envir" :key="name">
          {{ name }} {{ data }}
        </b-list-group-item>
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
        '현재위치 좌표 x: ': 1.123,
        '현재위치 좌표 y :': 2.4545,
        '배터리 상태 :': '97%',
        '전력 공급 상태 : ': 4,
        // 더 추가할 것이 있는지
      },
      envir: {
        '날짜 :': '9월 1일 13:00',
        '기온 :': '25도',
        '날씨 상태 :': '구름 많음',
      },
    };
  },
  // 찾는 거를 왜하냐? 1. 배터리 나갔을 수도 있는데... 근데 연결은 되지않나?
  // 이 부분 한번 같이 말해보기 ㅇㅇ
  methods: {
    changeRobotStatus() {
      this.checkRobot = false;
      console.log('왜안나와?');
    },
  },
  created() {
    // 아니면 css transition으로
    if (this.checkRobot) {
      setTimeout(() => {
        console.log('gdgd');
        this.changeRobotStatus();
      }, 3000);
    }
  },
};
</script>

<style scoped>
  .status {
    /* display: inline-block; */
    margin-top: 30%;
    padding: 10px;
    animation: fadein 2.5s;
    -webkit-animation: fadein 2.5s; /* Safari and Chrome */
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
