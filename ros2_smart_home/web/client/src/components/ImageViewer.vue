<template>
  <v-container>
    <v-btn
      @click="update"
    >
      readDir
    </v-btn>
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
  },
  methods: {
    update() {
      this.$socket.emit('back_getImgList_front', '1');
    },
    imgClick(event) {
      const { hostname } = document.location;
      const serverPort = 12001;
      const fileName = event.target.innerText;
      const imageSrc = `http://${hostname}:${serverPort}/images/${fileName}`;
      this.imgSrc = imageSrc;
      console.log(imageSrc);
    },
  },
};
</script>

<style>
#imgList {
    list-style-type: none;
}
</style>
