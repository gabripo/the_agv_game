FROM nginx:stable-alpine

COPY nginx.conf /etc/nginx/conf.d/default.conf

COPY index.html /usr/share/nginx/html/
COPY script.js /usr/share/nginx/html/
COPY design_choices.md /usr/share/nginx/html/
COPY implementation_plan.md /usr/share/nginx/html/
COPY README.md /usr/share/nginx/html/

EXPOSE 80

CMD ["nginx", "-g", "daemon off;"]
