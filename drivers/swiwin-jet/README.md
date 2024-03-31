# Script for control swiwin-jet turbine

## Default variables 
**For AP9**   
- rpm: engine RPM  
- EGT: engine temperature  
- user5: engine feedback throttle 
- Vm: engine voltage 

**For AP11**     
- sns::env::eng::rpm: engine RPM   
- sns::env::eng::egt: engine temperature
- sns::env::eng::voltage: engine voltage   
- sns::env::pwr::vsrv: RC voltage     
- est::env::usr::u2: pump voltage
- est::env::usr::u1: engine feedback throttle    

## Engine start procedure
- Power Engine ON (ctr.pwr.eng=on)
- Engine starter (ctr.eng.starter=start)
