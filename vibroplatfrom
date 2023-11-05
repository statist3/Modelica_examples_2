model AdvancedTripleSpringDamperSystem
  // Параметры виброопор: масса, жесткость, демпфирование и внешняя сила
  parameter Real massValue = 50; // масса платформы в килограммах
  parameter Real springStiffness[3] = {20000, 15000, 10000}; // жесткости виброопор в Н/м
  parameter Real dampingCoeff[3] = {100, 150, 200}; // коэффициенты демпфирования виброопор в Н*с/м
  parameter Real gravity = 9.81; // ускорение свободного падения в м/с^2
  parameter Real forceAmplitude = 500; // амплитуда внешней силы в Н
  parameter Real forceFrequency = 2; // частота внешней силы в Гц

  // Компоненты системы
  Modelica.Mechanics.Translational.Components.Mass mass(m=massValue);
  Modelica.Mechanics.Translational.Components.Spring spring1(c=springStiffness[1]);
  Modelica.Mechanics.Translational.Components.Damper damper1(d=dampingCoeff[1]);
  Modelica.Mechanics.Translational.Components.Spring spring2(c=springStiffness[2]);
  Modelica.Mechanics.Translational.Components.Damper damper2(d=dampingCoeff[2]);
  Modelica.Mechanics.Translational.Components.Spring spring3(c=springStiffness[3]);
  Modelica.Mechanics.Translational.Components.Damper damper3(d=dampingCoeff[3]);
  Modelica.Mechanics.Translational.Sources.Force force(
    f=forceAmplitude*sin(2*Modelica.Constants.pi*forceFrequency*time));
  Modelica.Mechanics.Translational.Components.Fixed fixed1;
  Modelica.Mechanics.Translational.Components.Fixed fixed2;
  Modelica.Mechanics.Translational.Components.Fixed fixed3;

  // Соединение компонентов и учет гравитации
equation
  // Связывание массы с пружинами и демпферами
  connect(mass.flange_b, spring1.flange_b);
  connect(spring1.flange_a, damper1.flange_a);
  connect(damper1.flange_b, fixed1.flange);

  connect(mass.flange_b, spring2.flange_b);
  connect(spring2.flange_a, damper2.flange_a);
  connect(damper2.flange_b, fixed2.flange);

  connect(mass.flange_b, spring3.flange_b);
  connect(spring3.flange_a, damper3.flange_a);
  connect(damper3.flange_b, fixed3.flange);

  // Учет внешней силы
  connect(force.flange, mass.flange_b);

  // Здесь больше не нужно явное уравнение для силы, так как оно генерируется автоматически

end AdvancedTripleSpringDamperSystem;
