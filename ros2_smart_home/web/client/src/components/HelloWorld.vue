<template>
  <v-row class="pa-3">
    <!-- 수동 컨트롤 버튼 생성 -->
    <v-col cols="6">
      <p>Manual Controller</p>
      <Move />
    </v-col>
    <!-- 카메라 이미지 뷰어 생성 -->
    <CameraView />
  </v-row>
</template>

<script>
import CameraView from './CameraView.vue';
import Move from './Move.vue';

export default {
  components: {
    CameraView,
    Move,
  },
  data() {
    return {
    };
  },
  methods: {},
  computed: {},
};
</script>

<style></style>
