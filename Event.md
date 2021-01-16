Event :

```matlab
event = eventGenerate(storageID,tag,delay,priority)

event = eventTimer(tag,delay)


```

新车 进入 storage3 ，巡航timestep后产生事件——向storage2中生成新的INFOEntity（产生完立马销毁），同时调用event action ：INFOGenerateImpl，该动作将一个INFOEntity送到output，进入COORDINATOR，COORDINATOR会对该INFOEntity编号，然后再送入storage2中，obj.newArrival = INFOEntity.data.VehicleID

简言之，新车进来后，送到COORDINATOR去编号，编完号送回storage2，更新newArrival变量值。相当于是新车先到COORDINATOR的注册，注册完了COORDINATOR会反馈给控制器刚刚注册好的那个编号是啥。