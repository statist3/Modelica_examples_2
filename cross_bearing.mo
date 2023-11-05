model CrossedShaftPlatform
  // Параметры сервоприводов и механики
  parameter Real k_shaft = 1e3 "Shaft stiffness [N/m]";
  parameter Real c_shaft = 10 "Shaft damping [N.s/m]";
  parameter Real J_shaft = 0.01 "Shaft inertia [kg.m^2]";
  parameter Real precision = 2 * (1/60) * (Modelica.Constants.pi/180) "Precision in radians";

  // Определение контроллеров PID для сервоприводов
  Modelica.Blocks.Continuous.PID controller1(k=100, Ti=0.5, Td=0.1);
  Modelica.Blocks.Continuous.PID controller2(k=100, Ti=0.5, Td=0.1);

  // Ограничители для контроллеров PID
  Modelica.Blocks.Nonlinear.Limiter limiter1(uMax=100, uMin=-100);
  Modelica.Blocks.Nonlinear.Limiter limiter2(uMax=100, uMin=-100);

  // Элементы преобразования сигнала в крутящий момент
  Modelica.Mechanics.Rotational.Sources.Torque torque1;
  Modelica.Mechanics.Rotational.Sources.Torque torque2;

  // Модели валов с инерцией и упругостью
  Modelica.Mechanics.Rotational.Components.Inertia shaft1(J=J_shaft);
  Modelica.Mechanics.Rotational.Components.Inertia shaft2(J=J_shaft);
  Modelica.Mechanics.Rotational.Components.SpringDamper damper1(c=c_shaft, d=k_shaft);
  Modelica.Mechanics.Rotational.Components.SpringDamper damper2(c=c_shaft, d=k_shaft);

  // Сенсоры углов для обратной связи к контроллерам
  Modelica.Mechanics.Rotational.Sensors.AngleSensor angleSensor1;
  Modelica.Mechanics.Rotational.Sensors.AngleSensor angleSensor2;

  // Входы для целевых углов (предположительно внешние входы)
  input Real targetAngle1;
  input Real targetAngle2;

initial equation
  // Начальные условия для углов и угловых скоростей валов
  shaft1.phi = 0;
  shaft2.phi = 0;
  shaft1.w = 0;
  shaft2.w = 0;

equation
  // Подключение сенсоров углов к валам
  connect(angleSensor1.flange, shaft1.flange_a);
  connect(angleSensor2.flange, shaft2.flange_a);

  // Рассчитываем ошибку управления и подаем на вход PID-контроллера
  controller1.u = targetAngle1 - angleSensor1.phi;
  controller2.u = targetAngle2 - angleSensor2.phi;
  
  // Ограничение выходного значения контроллеров PID и подключение к валам через элементы преобразования сигнала в крутящий момент
  connect(controller1.y, limiter1.u);
  connect(limiter1.y, torque1.tau);
  connect(torque1.flange, shaft1.flange_a);
  connect(controller2.y, limiter2.u);
  connect(limiter2.y, torque2.tau);
  connect(torque2.flange, shaft2.flange_a);

  // Установление связи между демпфирующими элементами и валами
  connect(damper1.flange_b, shaft1.flange_b);
  connect(damper2.flange_b, shaft2.flange_b);

end CrossedShaftPlatform;
