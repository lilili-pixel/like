    为实现电动汽车无线充电系统，本作品基于STM32G474，自制无线线圈和三维平台，搭建一套由DC-DC变换器组成的电动汽车无线充电桩系统。DC-DC变换器包括原边逆变器和副边整流桥，原边逆变器采用移相控制和PI算法进行调节，副边整流桥采用不控整流方案，贴合实际应用中的低成本和简便性。整个系统采用SS补偿结构，并匹配谐振网络参数，实现恒定电流输出。同时本项目还搭建一套三维平台进行X、Y、Z三轴移动模拟电动汽车无线充电场景，三维平台上放置发射线圈，采用机器视觉控制，可对接收线圈位置的自动追踪和校准，实现高效率的无线电能传输（WPT），并对电动汽车进行无线充电。
    本系统在阻感性负载下进行测试，实现了800W非接触式能量传输，整体效率达到90%。系统应用移相控制，使系统在输入电压和负载大小变化时，输出负载调整率小于0.1%，稳定10A恒流输出。三维平台能根据接收线圈进行自动调整，可实现1mm内误差的精准定位。
