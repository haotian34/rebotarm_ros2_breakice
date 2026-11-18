#!/usr/bin/env bash
set -e
# The installed service owns the process lifetime, including across browser reloads.
systemctl --user start rebotarm-studio.service
for STUDIO_TRY in {1..40}; do
  if /usr/bin/python3 -c 'import json,urllib.request; assert json.load(urllib.request.urlopen("http://127.0.0.1:8088/api/runtime",timeout=.5))["managed"]' >/dev/null 2>&1; then
    exec xdg-open 'http://127.0.0.1:8088'
  fi
  sleep .25
done
if command -v zenity >/dev/null 2>&1; then
  zenity --error --text='本地网页服务未能启动。请查看系统日志中的 reBot Arm Studio 服务。'
fi
exit 1
