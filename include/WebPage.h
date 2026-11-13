#ifndef WEBPAGE_H
#define WEBPAGE_H

#include <Arduino.h>

const char index_html[] PROGMEM = R"rawliteral(<!DOCTYPE html>
<html lang="vi"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Điều khiển xe 3 bánh</title>
<style>
:root{color-scheme:light;--bg:#eef1f3;--card:#fff;--tx:#17212b;--mu:#5d6b78;--ac:#0b7a75;--bd:#cdd5db;--er:#c42b1c;--wa:#9a5b00}
:root[data-theme=dark]{color-scheme:dark;--bg:#10161b;--card:#18212a;--tx:#e4eaef;--mu:#93a1ad;--ac:#2fb5ad;--bd:#2c3945;--er:#ff7b6e;--wa:#f2b44d}
*{box-sizing:border-box}
body{margin:0;padding:12px;font:15px/1.4 system-ui,"Segoe UI",sans-serif;background:var(--bg);color:var(--tx)}
.w{max-width:480px;margin:auto}
.top{display:flex;align-items:flex-start;justify-content:space-between;gap:10px;margin:2px 0 10px}
h1{font-size:19px;margin:0}
.mem{font-size:12.5px;color:var(--mu);margin-top:3px}
#th{flex-shrink:0;padding:7px 11px;border:1px solid var(--bd);border-radius:6px;background:var(--card);color:var(--tx);font:inherit;font-size:13px;cursor:pointer}
.card{background:var(--card);border:1px solid var(--bd);border-radius:8px;padding:12px;margin-bottom:10px}
.seg{display:flex;gap:6px;margin-bottom:10px}
#pn{display:grid;grid-template-columns:1fr 1fr}
.seg button{flex:1;padding:9px 4px;border:1px solid var(--bd);background:transparent;color:var(--tx);border-radius:6px;font:inherit}
.seg button.on{background:var(--ac);border-color:var(--ac);color:#fff;font-weight:600}
label{display:block;font-size:13px;color:var(--mu);margin:8px 0 3px}
input{width:100%;padding:9px;border:1px solid var(--bd);border-radius:6px;font:inherit;background:var(--bg);color:var(--tx)}
input::placeholder{color:var(--mu);opacity:.55}
input[readonly]{opacity:.55}
button:focus-visible,input:focus-visible{outline:2px solid var(--ac);outline-offset:2px}
.msg{min-height:20px;font-size:13px;margin-top:8px}
.er{color:var(--er)}.wa{color:var(--wa)}
.row{display:flex;gap:8px;margin-top:10px}
.row button{flex:1;padding:12px;border:0;border-radius:6px;font:inherit;font-weight:600;color:#fff}
#go{background:var(--ac)}#go:disabled{opacity:.35}
#st{background:var(--er)}
table{width:100%;border-collapse:collapse;font-size:14px;margin-top:6px}
td{padding:5px 0;border-bottom:1px solid var(--bd)}td+td{text-align:right;font-variant-numeric:tabular-nums}
#live{font-size:13px;color:var(--mu)}#live b{color:var(--tx)}
</style></head><body><div class="w">
<div class="top">
  <div>
    <h1>Xe tự hành và robot di động</h1>
    <div class="mem">Thành viên: An Hoàng Anh - Trần Hải Nam - Nguyễn Công Thành</div>
  </div>
  <button id="th" type="button" aria-label="Chuyển giao diện sáng/tối"></button>
</div>
<div class="seg" id="pn"></div>
<div class="card">
  <div class="seg" id="md"></div>
  <label id="la" for="a"></label><input id="a" placeholder="0.00" inputmode="decimal" autocomplete="off">
  <label id="lb" for="b"></label><input id="b" placeholder="0.00" inputmode="decimal" autocomplete="off">
  <label for="t">Thời gian chạy (s)</label><input id="t" placeholder="0.00" inputmode="decimal" autocomplete="off">
  <div class="msg" id="msg"></div>
  <div class="row"><button id="go">Chạy</button><button id="st">Dừng</button></div>
</div>
<div class="card"><b>Giá trị tính từ công thức</b><table id="out"><tr><td>Chưa có lệnh</td><td></td></tr></table></div>
<div class="card" id="live">Đang kết nối...</div>
<div class="card"><a href="/log" download="run.csv">Tải log lần chạy gần nhất (CSV)</a><span id="logn"></span></div>
</div>
<script>
const $=id=>document.getElementById(id);

// ---- Giao diện sáng / tối ----
function setTheme(t){
  document.documentElement.dataset.theme=t;
  $('th').textContent=t==='dark'?'☀ Sáng':'☾ Tối';   // nhãn = chế độ sẽ chuyển sang
  try{localStorage.setItem('theme',t);}catch(e){}
}
(function(){
  let s=null;try{s=localStorage.getItem('theme');}catch(e){}
  setTheme(s||(matchMedia('(prefers-color-scheme:dark)').matches?'dark':'light'));
  $('th').onclick=()=>setTheme(document.documentElement.dataset.theme==='dark'?'light':'dark');
})();

const LH=0.092, MAXWV=0.8, MAXT=30;
const P={
 fk:{n:'Động học thuận',pair:1,la:'ω_R: tốc độ góc bánh phải (rad/s)',lb:'ω_L: tốc độ góc bánh trái (rad/s)',ma:18,mb:18},
 ik:{n:'Động học nghịch',pair:0,la:'V_m: vận tốc tâm xe (m/s)',lb:'ω_m: vận tốc góc xe (rad/s)',ma:0.6,mb:3.5},
 fd:{n:'Động lực học thuận',pair:1,la:'τ_R: mô-men bánh phải (N·m)',lb:'τ_L: mô-men bánh trái (N·m)',ma:0.15,mb:0.15},
 id:{n:'Động lực học nghịch',pair:0,la:'V_m mục tiêu (m/s)',lb:'ω_m mục tiêu (rad/s)',ma:0.6,mb:3.5}
};
const M={straight:'Đi thẳng',spin:'Quay tại tâm',arc:'Đi vòng cung'};
let p='fk',m='straight';
const num=el=>el.value.trim()===''?NaN:parseFloat(el.value.replace(',','.'));
const eq=(x,y)=>Math.abs(x-y)<1e-6;

function segs(box,obj,cur,set){
  box.innerHTML='';
  for(const k in obj){
    const b=document.createElement('button');
    b.textContent=obj[k].n||obj[k];
    if(k===cur)b.className='on';
    b.onclick=()=>{set(k);fill();};
    box.appendChild(b);
  }
}
function fill(){           // đổi ô / đổi chế độ: nạp lại nhãn, xóa số đã nhập (hiện placeholder 0.00)
  segs($('pn'),P,p,k=>p=k);
  segs($('md'),M,m,k=>m=k);
  $('la').textContent=P[p].la;
  $('lb').textContent=P[p].lb;
  $('a').value='';$('b').value='';
  enforce();
}
// Ép dấu / khóa ô theo chế độ đang chọn
function enforce(){
  const A=$('a'),B=$('b'),T=$('t');
  [A,B,T].forEach(e=>{e.value=e.value.replace(/[^0-9.,\-]/g,'');});
  T.value=T.value.replace(/-/g,'');
  const pos=e=>e.value=e.value.replace(/-/g,'');
  // Mở khóa: ô đang bị khóa thì trả về trống (hiện placeholder)
  [A,B].forEach(e=>{if(e.dataset.lk){e.value='';delete e.dataset.lk;}e.readOnly=false;});
  const lock=(e,v)=>{e.value=v;e.readOnly=true;e.dataset.lk='1';};

  if(P[p].pair){
    if(m==='straight'){pos(A);lock(B,A.value);}      // hai bánh luôn bằng nhau và dương
    else if(m==='arc'){pos(A);pos(B);}
  }else{
    if(m==='straight'){pos(A);lock(B,'0');}          // omega_m = 0
    else if(m==='spin'){lock(A,'0');}                // V_m = 0
    else{pos(A);pos(B);}
  }
  refresh();
}
function check(){
  const a=num($('a')),b=num($('b')),t=num($('t')),q=P[p];
  if([a,b,t].some(isNaN))return'Nhập đủ 3 giá trị số.';
  if(t<0.1||t>MAXT)return'Thời gian phải trong khoảng 0.1 đến '+MAXT+' s.';
  if(Math.abs(a)>q.ma||Math.abs(b)>q.mb)return'Vượt giới hạn: |giá trị 1| ≤ '+q.ma+', |giá trị 2| ≤ '+q.mb+'.';
  if(q.pair){
    if(m==='straight'&&!(a>0&&eq(a,b)))return'Đi thẳng: hai giá trị phải bằng nhau và lớn hơn 0.';
    if(m==='spin'&&!(a*b<0))return'Quay tại tâm: hai giá trị phải trái dấu nhau.';
    if(m==='arc'&&!(a>0&&b>0&&!eq(a,b)))return'Đi vòng cung: hai giá trị phải dương và khác nhau.';
  }else{
    if(m==='straight'&&!(a>0&&b===0))return'Đi thẳng: V_m phải lớn hơn 0 và ω_m = 0.';
    if(m==='spin'&&!(a===0&&b!==0))return'Quay tại tâm: V_m = 0 và ω_m khác 0.';
    if(m==='arc'&&!(a>0&&b>0&&a>b*LH))return'Đi vòng cung: V_m, ω_m phải dương và V_m > ω_m·'+LH+' (hai bánh cùng quay tiến).';
    if(a+Math.abs(b)*LH>MAXWV)return'Vận tốc bánh ngoài vượt '+MAXWV+' m/s.';
  }
  return'';
}
function warn(){
  const a=num($('a')),b=num($('b'));
  if(P[p].pair&&m==='spin'&&!eq(Math.abs(a),Math.abs(b)))return'Lưu ý: hai giá trị khác độ lớn nên xe vừa tiến vừa xoay, không xoay đúng tại tâm.';
  if(p==='fd'&&(Math.abs(a)<0.06||Math.abs(b)<0.06))return'Lưu ý: mô-men dưới 0.06 N·m sẽ bị nâng lên PWM tối thiểu (dead-zone).';
  return'';
}
function refresh(){
  // Chưa nhập gì thì không hiện lỗi, chỉ khóa nút Chạy
  const touched=['a','b','t'].some(i=>$(i).value!==''&&!$(i).dataset.lk);
  const e=check(),w=e?'':warn();
  $('msg').className='msg '+(e?'er':'wa');
  $('msg').textContent=touched?(e||w):'';
  $('go').disabled=!!e;
}
['a','b','t'].forEach(i=>$(i).addEventListener('input',enforce));

$('go').onclick=async()=>{
  const q='panel='+p+'&mode='+m+'&a='+num($('a'))+'&b='+num($('b'))+'&t='+num($('t'));
  try{
    const j=await(await fetch('/run?'+q)).json();
    if(!j.ok){$('msg').className='msg er';$('msg').textContent=j.err;return;}
    $('out').innerHTML=j.out.map(o=>'<tr><td>'+o[0]+'</td><td>'+o[1]+' '+o[2]+'</td></tr>').join('');
  }catch(e){$('msg').className='msg er';$('msg').textContent='Mất kết nối với robot.';}
};
$('st').onclick=()=>fetch('/stop').catch(()=>{});

setInterval(async()=>{
  try{
    const s=await(await fetch('/status')).json();
    $('live').innerHTML='<b>'+(s.run?'Đang chạy, còn '+s.left.toFixed(1)+' s':'Đang dừng')+'</b><br>'+
      'θ (IMU) = '+s.theta.toFixed(1)+'°<br>'+
      'V = '+s.V.toFixed(3)+' m/s, ω = '+s.W.toFixed(2)+' rad/s<br>'+
      'v_R = '+s.vr.toFixed(3)+' m/s, v_L = '+s.vl.toFixed(3)+' m/s<br>'+
      'x = '+s.x.toFixed(2)+' m, y = '+s.y.toFixed(2)+' m';
    $('logn').textContent=' ('+s.log+' mẫu'+(s.rec?', đang ghi':'')+')';
  }catch(e){$('live').textContent='Mất kết nối với robot.';}
},300);
fill();
</script></body></html>)rawliteral";

#endif // WEBPAGE_H
