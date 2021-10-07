<template>
  <v-row>
    <v-row class="ma-0 pa-0">
      <v-col>
        <v-btn
          fab text small
          color="grey darken-2"
          @click="move(0)"

        >
          <v-icon small>
            mdi-chevron-up
          </v-icon>
        </v-btn>
      </v-col>
    </v-row>
    <v-row class="ma-0 pa-0">
      <v-col cols="4">
        <v-btn
          fab text small
          color="grey darken-2"
          @click="move(2)"
        >
          <v-icon small>
            mdi-chevron-left
          </v-icon>
        </v-btn>
      </v-col>
      <v-col cols="4">
        <v-btn
          fab text small
          color="grey darken-2"
          @click="move(1)"
        >
          <v-icon small>
            mdi-chevron-down
          </v-icon>
        </v-btn>
      </v-col>
          <v-col cols="4">
        <v-btn
          fab text small
          color="grey darken-2"
          @click="move(3)"
        >
          <v-icon small>
            mdi-chevron-right
          </v-icon>

        </v-btn>
      </v-col>
    </v-row>
       <v-btn
             fab text small
          color="grey darken-2"
   @click="stop">
     <v-icon small>
            mdi-chevron-right
          </v-icon></v-btn>
  </v-row>

</template>

<script>
export default {
  data() {
    return {
      direction: ['go', 'back', 'left', 'right'],
    };
  },
  mounted() {},
  methods: {
    move(index) {
      const message = this.direction[index];
      console.log(message);
      this.$socket.emit('back_movemanualy_front', message);
    },
    stop() {
      this.$socket.emit('back_movemanualy_front', 'stop');
    },
  },
};
</script>

<style>

</style>
