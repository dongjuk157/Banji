<template>
  <v-container>
    <v-row>
      <v-col
        cols=6
      >
      <p>가전</p>
      </v-col>
      <v-col
        cols=6
      >
      <p>ON/OFF</p>
      </v-col>
      <!-- <v-col
        cols=4
      >
      <p>Video</p>
      </v-col> -->
    </v-row>
    <hr>
    <v-row :id="iot" v-for="(iot, index) in myiot" :key=index>
      <v-col
        cols=6
      >
      <p>{{ iot[0] }}</p>
      </v-col>
      <v-col
        cols=6
      >
      <v-btn
        @click="control(index)"
        dark
        :color="btncolor[myiot[index][1]]"
      >
        {{ iot[1] }}
        <!-- {{ btncolor[myiot[index][1]] }} -->
      </v-btn>
      </v-col>
      <!-- <v-col
        cols=4
      >

      <v-btn
      color="green"><v-icon>mdi-turtle</v-icon><span style="color: white;">Video</span></v-btn>
      </v-col> -->
    </v-row>
  </v-container>

</template>

<script>
export default {
  data() {
    return {
      myiot: {
        1: ['에어컨', 'OFF'],
        2: ['공기청정기', 'OFF'],
        3: ['TV', 'OFF'],
        4: ['거실 조명', 'OFF'],
        5: ['거실 커튼', 'OFF'],
        6: ['내 방 조명', 'OFF'],
      },
      btncolor: {
        ON: 'primary',
        OFF: 'error',
      },
    };
  },
  mounted() {
    this.$socket.on('front_control_back', (message) => {
      // console.log('SEND!', message);
      // 받은 상태로 업데이트
      // const message = {
      //   index: idx,
      //   name: this.myiot[idx][0], // 에어컨
      //   status: this.myiot[idx][1], // on, off
      // };
      const { index, name, status } = message;
      const newData = [name, status];
      this.myiot[index] = newData;
    });
    // 가전 상태 받아오기
    // this.$socket.on('front_getIoTStatus_back', (message) => {
    //   // msg: array 형태?
    //   const initialIoT = {};
    //   message.forEach((item, idx) => {
    //     const { name, status } = item;
    //     initialIoT[idx + 1] = [name, status];
    //   });
    //   this.myiot = initialIoT;
    // });
    // this.getIoTDevices();
  },
  methods: {
    control(idx) {
      const toggle = this.myiot[idx][1] === 'ON' ? 'OFF' : 'ON';
      const data = {
        index: idx,
        name: this.myiot[idx][0], // 에어컨
        status: this.myiot[idx][1], // on, off
        control: toggle,
      };
      // console.log('control', data);
      this.$socket.emit('back_control_front', data);
    },
    // getIoTDevices() {
    //   const data = { key: false };
    //   this.$socket.emit('back_getIoTDevices_front', data);
    // },
  },
};
</script>
<style scoped>

</style>
