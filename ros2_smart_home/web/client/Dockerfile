FROM node:14 as build-stage
WORKDIR /var/jenkins_home/workspace/B301/client

COPY package*.json ./
RUN npm install 
COPY . .
RUN npm run build



FROM nginx:stable-alpine as production-stage
COPY --from=build-stage /var/jenkins_home/workspace/B301/client/dist /usr/share/nginx/html
COPY --from=build-stage /var/jenkins_home/workspace/cicd_pipeline_test/client/nginx/conf.d/*.conf /etc/nginx/conf.d/

EXPOSE 80
CMD ["nginx", "-g", "daemon off;"]
