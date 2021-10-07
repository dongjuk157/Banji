<template>
  <v-container>
    <ul id="imgList">
      <li v-for="img in imgList" :key="img">
        <v-dialog
          v-model="dialog"
        >
          <template v-slot:activator="{ on, attrs }">
            <div
              v-bind="attrs"
              v-on="on"
              @click="imgClick"
              class="ma-2"
            >
              {{ img }}
            </div>
          </template>
          <v-card>
            <v-img :src="imgSrc"></v-img>
          </v-card>
        </v-dialog>
      </li>
    </ul>
  </v-container>
</template>

<script>
export default {
  data() {
    return {
      imgList: [],
      imgSrc: null,
      dialog: false,
    };
  },
  mounted() {
    this.$socket.on('front_getImgList_back', (msg) => {
      this.imgList = msg;
    });
    this.update();
  },
  methods: {
    update() {
      this.$socket.emit('back_getImgList_front', 'screenshot');
    },
    imgClick(event) {
      const { hostname } = document.location;
      const serverPort = 12001;
      const fileName = event.target.innerText;
      const imageSrc = `http://${hostname}:${serverPort}/images/screenshot/${fileName}`;
      this.imgSrc = imageSrc;
    },
  },
};
</script>

<style>
#imgList {
    list-style-type: none;
}
</style>
