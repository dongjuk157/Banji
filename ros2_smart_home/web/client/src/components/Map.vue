/* eslint-disable comma-dangle */
<template>
  <div>
    <div>
      <canvas
        id="canvas"
        @click="clickCanvas"
        width="350" height="350"
      >
      </canvas>
    </div>
  </div>
</template>

<script>
// import json from './json/map.json';

export default {
  props: {
    canClick: {
      type: Boolean,
      default: true,
    },
  },
  data() {
    return {
      MAP: [],
      cellStyle: 'cell',
      temp: [],
      pos: [],
      ctx: [],
    };
  },
  mounted() {
    this.ctx = document.getElementById('canvas').getContext('2d');
    this.$socket.on('front_position_back', (mes) => {
      this.ctx.fillStyle = 'rgb('.concat('100', ',', '100', ',', '100', ')');
      this.ctx.fillRect(this.pos[0], this.pos[1], 5, 5);
      this.pos = mes;
    });
  },
  watch: {
    temp() {
      this.MAP = this.temp;
    },
    pos() {
      this.ctx.fillStyle = 'rgb('.concat('255', ',', '10', ',', '10', ')');
      this.ctx.fillRect(this.pos[0], this.pos[1], 5, 5);
    },
  },
  methods: {
    draw() {
      this.ctx = document.getElementById('canvas').getContext('2d');
      let color = String();
      let idx = 0;
      for (let i = 0; i < 350; i += 1) {
        for (let j = 0; j < 350; j += 1) {
          color = String(100 - Math.floor(this.MAP[idx]));
          this.ctx.fillStyle = 'rgb('.concat('0', ',', color, ',', color, ')');
          this.ctx.fillRect(j, i, 1, 1);
          idx += 1;
        }
      }
    },
    request_pos() {
      setInterval(() => {
        this.$socket.emit('back_position_front', { key: 1 });
      }, 500);
    },
    loadMap() {
      this.$socket.emit('back_loadmap_front', { key: 1 });
    },
    start() {
      this.$socket.on('front_loadmap_back', (mes) => {
        this.MAP = mes;
        this.draw();
      });
      this.loadMap();
    },
    clickCanvas(event) {
      if (this.canClick) {
        const message = [event.offsetX, event.offsetY];
        this.ctx.fillStyle = 'rgb('.concat('100', ',', '230', ',', '100', ')');
        this.ctx.fillRect(event.offsetX, event.offsetY, 5, 5);
        this.$socket.emit('back_move_front', message);
      }
    },
  },
  created() {
    this.start();
    this.request_pos();
  },
};
</script>

<style>

</style>
