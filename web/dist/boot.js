// Keep the entrypoint tiny so an app/module error cannot leave an endless spinner.
const revision='20261117-manual-6';
const reconnect=document.getElementById('rviz-reconnect');
if(reconnect)reconnect.onclick=()=>location.reload();
try{
 await import(`./app.js?v=${revision}`);
 document.documentElement.dataset.boot='ready';
}catch(error){
 document.documentElement.dataset.boot='failed';
 const placeholder=document.getElementById('rviz-placeholder');
 if(placeholder){placeholder.hidden=false;placeholder.querySelector('.loading-ring')?.remove();}
 const message=document.getElementById('rviz-message'),detail=document.getElementById('rviz-detail');
 if(message)message.textContent='网页初始化失败';
 if(detail)detail.textContent=`${error.message}。请点击下方“重新连接”重新加载。`;
 console.error('reBot Arm Studio startup failed:',error);
}
