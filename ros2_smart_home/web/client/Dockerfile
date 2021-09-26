FROM node:14 as build-stage
WORKDIR /var/jenkins_home/workspace/B301/ros2_smart_home/web/client/

# COPY package*.json ./
COPY package.json ./
RUN npm install 
COPY . .
RUN npm run build


FROM nginx:stable-alpine as production-stage
COPY --from=build-stage ./dist /usr/share/nginx/html
COPY --from=build-stage ./nginx/conf.d/*.conf /etc/nginx/conf.d/

EXPOSE 80
CMD ["nginx", "-g", "daemon off;"]