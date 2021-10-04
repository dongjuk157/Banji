<template>
  <div>
    <div>
      <canvas id="canvas" width="350" height="350">
      </canvas>
    </div>
  </div>
</template>

<script>
// import json from './json/map.json';

export default {
  data() {
    return {
      isloaded: false,
      mapArr: [],
    };
  },
  mounted() {
    this.$socket.on('front_loadmap_back', (message) => {
      // message: String converted text file
      // console.log(message);
      this.isloaded = true;
      const mapArr = message.split(' ');
      this.mapArr = mapArr;
      console.log(mapArr);
    });
    this.loadMap();
    this.draw();
  },
  methods: {
    loadMap() {
      this.$socket.emit('back_loadmap_front', { key: 1 });
    },
    draw() {
      const ctx = document.getElementById('canvas').getContext('2d');
      let color = String();
      let idx = 0;
      for (let i = 0; i < 350; i += 1) {
        for (let j = 0; j < 350; j += 1) {
          color = String(100 - Math.floor(this.mapArr[idx]));
          ctx.fillStyle = 'rgb('.concat(color, ',', color, ',', color, ')');
          ctx.fillRect(j, i, 1, 1);
          idx += 1;
        }
      }
    },
  },
};

</script>

<style>
</style>
