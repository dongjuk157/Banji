<template>
  <div class="pa-5">
    <v-row>
      <v-col cols="12" class="mb-4">
        <v-sheet height="64">
          <v-toolbar
            flat
          >
            <v-toolbar-title v-if="$refs.calendar">
              {{ $refs.calendar.title }}
            </v-toolbar-title>
            <v-col cols="4">
              <v-btn
                fab
                text
                small
                color="grey darken-2"
                @click="prev"
              >
                <v-icon small>
                  mdi-chevron-left
                </v-icon>
              </v-btn>
            </v-col>
            <v-col cols="4">
              <v-btn
                fab
                text
                small
                color="grey darken-2"
                @click="setToday"
              >
                <v-icon small>
                  mdi-checkbox-blank-circle-outline
                </v-icon>
              </v-btn>
            </v-col>
            <v-col cols="4">
              <v-btn
                fab
                text
                small
                color="grey darken-2"
                @click="next"
              >
                <v-icon small>
                  mdi-chevron-right
                </v-icon>
              </v-btn>
            </v-col>
            <v-spacer></v-spacer>
          </v-toolbar>
        </v-sheet>
        <v-sheet height="400">
          <v-calendar
            ref="calendar"
            v-model="focus"
            color="primary"
            :events="events"
            :event-color="getEventColor"
            :type="type"
            @click:more="viewDay"
            @click:date="viewDay"
            @change="update"
          ></v-calendar>
        </v-sheet>
      </v-col>
    </v-row>
    <v-dialog
      v-model="dialog"
      width="500"
    >
      <template v-slot:activator="{ on, attrs }">
        <v-btn
          fab
          text
          small
          color="grey darken-2"
          v-bind="attrs"
          v-on="on"
        >
          <v-icon small>
            mdi-plus
          </v-icon>
        </v-btn>
      </template>

      <v-card>
        <v-card-title class="text-h5 grey lighten-2">
          일정 추가
        </v-card-title>
        <v-card-text>
          <v-row>
            <v-col cols="12">
              <v-text-field
                v-model="schedule.name"
                label="내용"
              ></v-text-field>
            </v-col>
            <v-col cols="12" md="6">
              <v-text-field
                v-model="schedule.start.date"
                label="시작일"
                type="date"
              ></v-text-field>
            </v-col>
            <v-col cols="12" md="6">
              <v-text-field
                v-model="schedule.start.time"
                label="시작시간"
                type="time"
              ></v-text-field>
            </v-col>
            <v-col cols="12" md="6">
              <v-text-field
                v-model="schedule.end.date"
                label="종료일"
                type="date"
              ></v-text-field>
            </v-col>
            <v-col cols="12" md="6">
              <v-text-field
                v-model="schedule.end.time"
                label="종료시간"
                type="time"
              ></v-text-field>
            </v-col>
          </v-row>
        </v-card-text>
        <v-divider></v-divider>
        <v-card-actions>
          <v-spacer></v-spacer>
          <v-btn
            color="primary"
            text
            @click="addSchedule"
          >
            ADD
          </v-btn>
        </v-card-actions>
      </v-card>
    </v-dialog>

  </div>
</template>

<script>
export default {
  data: () => ({
    schedule: {
      name: '',
      start: {
        date: '',
        time: '',
      },
      end: {
        data: '',
        time: '',
      },
    },
    dialog: false,
    focus: '',
    type: 'month',
    typeToLabel: {
      month: 'Month',
      day: 'Day',
    },
    selectedEvent: {},
    selectedElement: null,
    selectedOpen: false,
    events: [],
    colors: ['blue', 'indigo', 'deep-purple', 'cyan', 'green', 'orange', 'grey darken-1'],
    // names: ['Meeting', 'Holiday', 'PTO', 'Travel', 'Event', 'Birthday', 'Conference', 'Party'],
  }),
  mounted() {
    this.$refs.calendar.checkChange();
    this.loadLocalstorage();
  },
  methods: {
    viewDay({ date }) {
      this.focus = date;
      this.type = 'day';
    },
    getEventColor(event) {
      return event.color;
    },
    setToday() {
      this.focus = '';
      this.type = 'month';
    },
    prev() {
      this.$refs.calendar.prev();
    },
    next() {
      this.$refs.calendar.next();
    },
    showEvent({ nativeEvent, event }) {
      const open = () => {
        this.selectedEvent = event;
        this.selectedElement = nativeEvent.target;
        requestAnimationFrame(() => requestAnimationFrame(() => {
          this.selectedOpen = true;
        }));
      };

      if (this.selectedOpen) {
        this.selectedOpen = false;
        requestAnimationFrame(() => requestAnimationFrame(() => open()));
      } else {
        open();
      }

      nativeEvent.stopPropagation();
    },
    update() {
      const events = [];
      // const min = new Date(`${start.date}T00:00:00`);
      // const max = new Date(`${end.date}T23:59:59`);
      // const days = (max.getTime() - min.getTime()) / 86400000;
      // const eventCount = this.rnd(days, days + 20);
      // for (let i = 0; i < eventCount; i += 1) {
      //   const allDay = this.rnd(0, 3) === 0;
      //   const firstTimestamp = this.rnd(min.getTime(), max.getTime());
      //   const first = new Date(firstTimestamp - (firstTimestamp % 900000));
      //   const secondTimestamp = this.rnd(2, allDay ? 288 : 8) * 900000;
      //   const second = new Date(first.getTime() + secondTimestamp);

      //   events.push({
      //     name: this.names[this.rnd(0, this.names.length - 1)],
      //     start: first,
      //     end: second,
      //     color: this.colors[this.rnd(0, this.colors.length - 1)],
      //     timed: !allDay,
      //   });
      // }

      this.events = events;
      this.loadLocalstorage();
    },
    rnd(a, b) {
      return Math.floor((b - a + 1) * Math.random()) + a;
    },
    addSchedule() {
      const startString = this.schedule.start.time ? `${this.schedule.start.date} ${this.schedule.start.time}` : this.schedule.start.date;
      const endString = this.schedule.end.time ? `${this.schedule.end.date} ${this.schedule.end.time}` : this.schedule.end.date;
      const item = {
        name: this.schedule.name,
        start: startString,
        end: endString,
        color: this.colors[this.rnd(0, this.colors.length - 1)],
        timed: this.schedule.start.time && this.schedule.end.time,
      };
      // 0. check all inputs right
      if (!this.schedule.name
        || !this.schedule.start.date
        || !this.schedule.end.date) {
        return;
      }
      // 1. add
      const events = [...this.events];
      events.push(item);
      this.events = events;
      // 1-1. save localstorage
      this.saveLocalstorage();
      // 2. initialize
      const initialSchedule = {
        name: '',
        start: {
          date: '',
          time: '',
        },
        end: {
          data: '',
          time: '',
        },
      };
      this.schedule = initialSchedule;
      // 3. dialog close
      this.dialog = false;
    },
    saveLocalstorage() {
      localStorage.setItem('events', JSON.stringify(this.events));
    },
    loadLocalstorage() {
      const eventsString = localStorage.getItem('events');
      const events = JSON.parse(eventsString);
      if (events) {
        this.events = events;
      }
    },
  },
};
</script>

<style>

</style>
