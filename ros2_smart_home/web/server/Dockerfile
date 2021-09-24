FROM node:14
WORKDIR /var/jenkins_home/workspace/B301/ros2_smart_home/web/server 

# 앱 의존성 설치
COPY package*.json ./

RUN npm install 
# 프로덕션을 위한 코드
# RUN npm ci --only=production

COPY . .

# binding port 3000
EXPOSE 3000

CMD ["npm", "start"]