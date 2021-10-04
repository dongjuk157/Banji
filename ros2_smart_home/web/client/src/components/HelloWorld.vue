<template>
  <v-container>
    <v-row class="pa-3">
      <!-- 수동 컨트롤 버튼 생성 -->
      <v-col cols="6">
        <p>Manual Controller</p>
        <Move />
      </v-col>
      <!-- 카메라 이미지 뷰어 생성 -->
      <CameraView />
      <!-- 저장된 이미지 리스트 출력 및 파일 뷰어-->
      <ImageViewer />
    </v-row>
  </v-container>
</template>

<script>
import CameraView from './CameraView.vue';
import ImageViewer from './ImageViewer.vue';
import Move from './Move.vue';

export default {
  components: {
    CameraView,
    Move,
    ImageViewer,
  },
  data() {
    return {
    };
  },
  mounted() {},
  methods: {},
  computed: {},
};
</script>

<style>
</style>
