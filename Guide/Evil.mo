within ;
package Evil
  model Original
    parameter Real myParameter=30 "example parameter";
    Real a;
    inner IDEAS.BoundaryConditions.SimInfoManager sim annotation (Placement(transformation(extent={{-84,70},{-64,90}})));
  protected
    IDEAS.Buildings.Components.Window windowN(
      redeclare IDEAS.Buildings.Data.Glazing.Ins2 glazing,
      inc=IDEAS.Types.Tilt.Wall,
      azi=IDEAS.Types.Azimuth.N,
      redeclare IDEAS.Buildings.Data.Frames.Aluminium fraType,
      redeclare IDEAS.Buildings.Components.Shading.None shaType,
      A=3)                                                       annotation (Placement(transformation(
          extent={{-6,-10},{6,10}},
          rotation=270,
          origin={-54,32})));
  public
    IDEAS.Buildings.Components.Zone
                    hell(
      nSurf=10,
      V=3.3*10*10,
      hZone=3.3,
      redeclare package Medium = IDEAS.Media.Air,
      redeclare IDEAS.Buildings.Components.InterzonalAirFlow.AirTight interzonalAirFlow)
                                                  annotation (Placement(transformation(extent={{-70,-40},{-10,16}})));
  protected
    IDEAS.Buildings.Components.OuterWall outerWallN(
      inc=IDEAS.Types.Tilt.Wall,
      azi=IDEAS.Types.Azimuth.N,
      A=3*10,
      redeclare IDEAS.Buildings.Data.Constructions.CavityWall constructionType,
      hasBuildingShade=false) annotation (Placement(transformation(
          extent={{-6,-10},{6,10}},
          rotation=270,
          origin={-28,32})));
    IDEAS.Buildings.Components.OuterWall outerWallE(
      inc=IDEAS.Types.Tilt.Wall,
      A=3*10,
      redeclare IDEAS.Buildings.Data.Constructions.CavityWall constructionType,
      hasBuildingShade=false,
      azi=IDEAS.Types.Azimuth.E) annotation (Placement(transformation(
          extent={{-6,-10},{6,10}},
          rotation=180,
          origin={8,4})));
    IDEAS.Buildings.Components.OuterWall outerWallS(
      inc=IDEAS.Types.Tilt.Wall,
      A=3*10,
      redeclare IDEAS.Buildings.Data.Constructions.CavityWall constructionType,
      hasBuildingShade=false,
      azi=IDEAS.Types.Azimuth.S) annotation (Placement(transformation(
          extent={{-6,-10},{6,10}},
          rotation=90,
          origin={-28,-56})));
    IDEAS.Buildings.Components.OuterWall outerWallW(
      inc=IDEAS.Types.Tilt.Wall,
      A=3*10,
      redeclare IDEAS.Buildings.Data.Constructions.CavityWall constructionType,
      hasBuildingShade=false,
      azi=IDEAS.Types.Azimuth.W) annotation (Placement(transformation(
          extent={{-6,-10},{6,10}},
          rotation=0,
          origin={-90,2})));
    IDEAS.Buildings.Components.Window windowE(
      redeclare IDEAS.Buildings.Data.Glazing.Ins2 glazing,
      inc=IDEAS.Types.Tilt.Wall,
      redeclare IDEAS.Buildings.Data.Frames.Aluminium fraType,
      redeclare IDEAS.Buildings.Components.Shading.None shaType,
      azi=IDEAS.Types.Azimuth.E,
      A=3)                       annotation (Placement(transformation(
          extent={{-6,-10},{6,10}},
          rotation=180,
          origin={8,-26})));
    IDEAS.Buildings.Components.Window windowE1(
      redeclare IDEAS.Buildings.Data.Glazing.Ins2 glazing,
      inc=IDEAS.Types.Tilt.Wall,
      redeclare IDEAS.Buildings.Data.Frames.Aluminium fraType,
      redeclare IDEAS.Buildings.Components.Shading.None shaType,
      azi=IDEAS.Types.Azimuth.S,
      A=3)                       annotation (Placement(transformation(
          extent={{-6,-10},{6,10}},
          rotation=90,
          origin={-54,-58})));
    IDEAS.Buildings.Components.Window windowE2(
      redeclare IDEAS.Buildings.Data.Glazing.Ins2 glazing,
      inc=IDEAS.Types.Tilt.Wall,
      redeclare IDEAS.Buildings.Data.Frames.Aluminium fraType,
      redeclare IDEAS.Buildings.Components.Shading.None shaType,
      azi=IDEAS.Types.Azimuth.W,
      A=3)                       annotation (Placement(transformation(
          extent={{-6,-10},{6,10}},
          rotation=0,
          origin={-90,-24})));
    IDEAS.Buildings.Components.OuterWall outerWallCeiling(
      redeclare IDEAS.Buildings.Data.Constructions.CavityWall constructionType,
      hasBuildingShade=false,
      inc=IDEAS.Types.Tilt.Ceiling,
      azi=IDEAS.Types.Azimuth.S,
      A=100) annotation (Placement(transformation(
          extent={{-6,-10},{6,10}},
          rotation=270,
          origin={-42,54})));
    IDEAS.Buildings.Components.OuterWall outerWallFloor(
      redeclare IDEAS.Buildings.Data.Constructions.CavityWall constructionType,
      hasBuildingShade=false,
      azi=IDEAS.Types.Azimuth.S,
      inc=IDEAS.Types.Tilt.Floor,
      A=100) annotation (Placement(transformation(
          extent={{-6,-10},{6,10}},
          rotation=90,
          origin={-40,-76})));

  public
    parameter Modelica.SIunits.Temp_C prot1rescued=21 "protected component";
    Components.EvilHeating evil annotation (Placement(transformation(extent={{84,-30},{130,20}})));

  protected
    Components.ProtectedTest protectedTest(prot1=prot1rescued) annotation (Placement(transformation(extent={{88,44},{128,84}})));
  equation
    a=myParameter*sin(myParameter)+5;
    connect(outerWallCeiling.propsBus_a,hell. propsBus[1]);
    connect(windowN.propsBus_a,hell. propsBus[2]);
    connect(outerWallN.propsBus_a,hell. propsBus[3]);
    connect(outerWallE.propsBus_a,hell. propsBus[4]);
    connect(windowE.propsBus_a,hell. propsBus[5]);
    connect(outerWallS.propsBus_a,hell. propsBus[6]);
    connect(windowE1.propsBus_a,hell. propsBus[7]);
    connect(windowE2.propsBus_a,hell. propsBus[8]);
    connect(outerWallW.propsBus_a,hell. propsBus[9]);
    connect(outerWallFloor.propsBus_a,hell. propsBus[10]);
    connect(evil.Tsensor[1], hell.TSensor) annotation (Line(points={{82.16,-19},{35.6,-19},{35.6,-6.4},{-7,-6.4}}, color={0,0,127}));
    connect(hell.gainCon, evil.port_a[1]) annotation (Line(points={{-10,-20.4},{-10,3},{79.86,3}}, color={191,0,0}));
    annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{180,100}}), graphics={Polygon(points={{-76,68},{-82,0},{-58,26},{-22,26},{-22,-76},{-60,-92},{34,-90},{8,-78},{2,
                24},{26,28},{40,4},{34,66},{-76,68}}, lineColor={28,108,200}), Text(
            extent={{10,2},{202,-74}},
            lineColor={28,108,200},
            textString="est",
            horizontalAlignment=TextAlignment.Left)}),
                                              Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{180,100}})),
      experiment(StopTime=25920000),
      __Dymola_experimentSetupOutput,
      __Dymola_experimentFlags(
        Advanced(
          EvaluateAlsoTop=false,
          GenerateVariableDependencies=false,
          OutputModelicaCode=false),
        Evaluate=true,
        OutputCPUtime=true,
        OutputFlatModelica=false));
  end Original;

  package Components

    model EvilWellMixedAir "Zone air model from IDEAS 2.0 with Enthalpy sensors. Slower simulations"
      extends IDEAS.Buildings.Components.ZoneAirModels.BaseClasses.PartialAirModel(
        final nSeg=1,
        mSenFac(min=0)=5);

        constant StateSelect stateSelectTVol = StateSelect.avoid "Set to .prefer to use temperature as a state in mixing volume";

      IDEAS.Fluid.Sensors.EnthalpyFlowRate senEntropy[nPorts + 2](                  redeclare package Medium = Medium, each m_flow_nominal=1)
        annotation (Placement(transformation(
            extent={{8,-8},{-8,8}},
            rotation=90,
            origin={0,50})));
      Modelica.Blocks.Interfaces.RealOutput heatGainsHC "Heat gains and loses in kW" annotation (Placement(transformation(extent={{100,-86},{120,-66}})));
      Modelica.Blocks.Interfaces.RealOutput EnergyGainsH "Heat gains in kWh"         annotation (Placement(transformation(extent={{-10,-10},{10,10}},
            rotation=270,
            origin={-54,-98})));
      Modelica.Blocks.Interfaces.RealOutput EnergyGainsC "Heat gains in kWh" annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=270,
            origin={-88,-98})));
      Modelica.Blocks.Continuous.Integrator EnergyGainsC1(k=1/3600) "Heat gains in kWh" annotation (Placement(transformation(
            extent={{5,-5},{-5,5}},
            rotation=0,
            origin={-39,-79})));
      Modelica.Blocks.Continuous.Integrator EnergyGainsC2(k=1/3600) "Heat gains in kWh" annotation (Placement(transformation(
            extent={{5,-5},{-5,5}},
            rotation=0,
            origin={-77,-71})));
      Modelica.Blocks.Math.Min EnergyGainsC3 "Heat gains in kWh" annotation (Placement(transformation(
            extent={{5,-5},{-5,5}},
            rotation=0,
            origin={-57,-59})));
      Modelica.Blocks.Math.Max EnergyGainsC4 "Heat gains in kWh" annotation (Placement(transformation(
            extent={{5,-5},{-5,5}},
            rotation=0,
            origin={-15,-81})));
      Modelica.Blocks.Sources.RealExpression realExpression annotation (Placement(transformation(extent={{-28,-56},{-8,-36}})));
    protected
      constant Modelica.SIunits.SpecificEnthalpy lambdaWater = Medium.enthalpyOfCondensingGas(T=273.15+35)
        "Latent heat of evaporation water";
      constant Boolean hasVap = Medium.nXi>0
        "Medium has water vapour";
      constant Boolean hasPpm = Medium.nC>0
        "Medium has trace substance";
      MixingVolumeNominal       vol(
        redeclare package Medium = Medium,
        energyDynamics=energyDynamics,
        massDynamics=massDynamics,
        p_start=p_start,
        T_start=T_start,
        X_start=X_start,
        C_start=C_start,
        C_nominal=C_nominal,
        allowFlowReversal=allowFlowReversal,
        V=Vtot,
        mSenFac=mSenFac,
        U_nominal=mSenFac*10*Vtot*1.2*1000,
        use_C_flow=true,
        nPorts=(2 + (if hasVap then 1 else 0) + (if hasPpm then 1 else 0))+nPorts,
        m_flow_nominal=0.1,
        T(stateSelect=stateSelectTVol))  annotation (Placement(
            transformation(
            extent={{-10,-10},{10,10}},
            rotation=180,
            origin={0,0})));

      Modelica.Thermal.HeatTransfer.Sensors.TemperatureSensor senTem
        annotation (Placement(transformation(extent={{10,-50},{30,-70}})));
      Modelica.Blocks.Math.Gain gaiLat(k=lambdaWater)
        "Gain for computing latent heat flow rate based on water vapor mass flow rate"
        annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=270,
            origin={64,58})));
      Modelica.Thermal.HeatTransfer.Sources.PrescribedHeatFlow preHeaFloLat(final
          alpha=0)
        "Prescribed heat flow rate for latent heat gain corresponding to water vapor mass flow rate"
        annotation (Placement(transformation(extent={{-10,-10},{10,10}},
            rotation=270,
            origin={64,22})));
    protected
      IDEAS.Fluid.Sensors.RelativeHumidity senRelHum(
        redeclare package Medium = Medium) if hasVap
        "Relative humidity of the zone air"
        annotation (Placement(transformation(extent={{30,-30},{50,-50}})));
        model MixingVolumeNominal
          "To avoid warning when modifying parameters of protected submodel dynBal of MixingVolumeMoistAir"
          parameter Modelica.SIunits.Energy U_nominal = mSenFac*10*m_nominal*1000 "Nominal value of internal energy";
          parameter Modelica.SIunits.Mass m_nominal = V*1.2 "Nominal value of internal energy";
          parameter Real[Medium.nXi] mXi_nominal = m_nominal*Medium.X_default[1:Medium.nXi] "Nominal value of internal energy";
          parameter Real[Medium.nC] mC_nominal = m_nominal*0.0015*ones(Medium.nC) "Nominal value of internal energy";
          extends IDEAS.Fluid.MixingVolumes.MixingVolumeMoistAir(
            mSenFac(min=0),
            dynBal(
              U(nominal=U_nominal),
              mC(nominal=mC_nominal),
              mXi(nominal=mXi_nominal),
              m(nominal=m_nominal)));
        end MixingVolumeNominal;
      IDEAS.Fluid.Sensors.PPM senPPM(
        redeclare package Medium = Medium) if hasPpm
        "CO2 sensor"
        annotation (Placement(transformation(extent={{50,-10},{70,-30}})));
    equation
      if hasVap then
        assert(vol.ports[1].Xi_outflow[1] <= 0.1,
               "The water content of the zone air model is very high. 
           Possibly you are simulating occupants (that generates a latent heat load), 
           but air is not being refreshed (for instance using ventilation or air leakage models)?",
               level=AssertionLevel.warning);
      else
        phi=0;
      end if;

      if not hasPpm then
        ppm=0;
      end if;

      E=vol.U;
      QGai=preHeaFloLat.Q_flow;
      for i in 1:nSurf loop
        connect(vol.heatPort, ports_surf[i]) annotation (Line(points={{10,-1.33227e-15},{10,-20},{-60,-20},{-60,0},{-100,0}},
                                                   color={191,0,0}));
      end for;
      for i in 1:nSeg loop
        connect(ports_air[i], vol.heatPort) annotation (Line(points={{100,0},{20,0},
                {20,-20},{10,-20},{10,0}},
                                       color={191,0,0}));
      end for;
      connect(senTem.port, vol.heatPort) annotation (Line(points={{10,-60},{10,0}},
                                  color={191,0,0}));
      connect(senTem.T,TAir)
        annotation (Line(points={{30,-60},{110,-60}},          color={0,0,127}));
      connect(vol.mWat_flow, mWat_flow) annotation (Line(points={{12,-8},{16,-8},{
              16,80},{108,80}},
                             color={0,0,127}));
      connect(vol.C_flow[1:Medium.nC], C_flow[1:Medium.nC]) annotation (Line(points={{12,6},{
              18,6},{18,40},{108,40}},
                    color={0,0,127}));
      connect(gaiLat.y, preHeaFloLat.Q_flow)
        annotation (Line(points={{64,47},{64,32}}, color={0,0,127}));
      connect(gaiLat.u, mWat_flow)
        annotation (Line(points={{64,70},{64,80},{108,80}}, color={0,0,127}));
      connect(preHeaFloLat.port, vol.heatPort) annotation (Line(points={{64,12},{64,
              0},{20,0},{20,-20},{10,-20},{10,0}},
                                     color={191,0,0}));
      connect(senRelHum.phi, phi)
        annotation (Line(points={{51,-40},{110,-40}},          color={0,0,127}));

    connect(senEntropy[1].port_a, port_a);
    connect(senEntropy[2].port_a, port_b);
    connect(senEntropy[3:nPorts+2].port_a, ports[1:nPorts]);
    connect(senEntropy[1].port_b, vol.ports[1]);
    connect(senEntropy[2].port_b, vol.ports[2]);
    connect(senEntropy[3:nPorts+2].port_b, vol.ports[3:nPorts+2]);

      heatGainsHC = (sum(senEntropy.H_flow) + sum(ports_surf.Q_flow) + sum(preHeaFloLat.Q_flow))/1000;

      //With this one it is possible to see the error (it will oscilate around 0 Watts):
      //heatGainsHC = sum(senEntropy.H_flow) + sum(ports_surf.Q_flow) + sum(ports_air.Q_flow)+sum(preHeaFloLat.Q_flow);


      connect(senRelHum.port, vol.ports[nPorts+3]) annotation (Line(points={{40,-30},
              {40,10},{1.33227e-15,10}},
                              color={0,127,255}));
      connect(senPPM.port, vol.ports[nPorts+3+(if hasVap then 1 else 0)]) annotation (Line(points={{60,-10},
              {60,10},{1.33227e-15,10}},
                              color={0,127,255}));

      connect(senPPM.ppm, ppm)
        annotation (Line(points={{71,-20},{110,-20}}, color={0,0,127}));
      connect(EnergyGainsC1.y, EnergyGainsH) annotation (Line(points={{-44.5,-79},{-54,-79},{-54,-98}}, color={0,0,127}));
      connect(EnergyGainsC2.y, EnergyGainsC) annotation (Line(points={{-82.5,-71},{-88,-71},{-88,-98}}, color={0,0,127}));
      connect(EnergyGainsC1.u, EnergyGainsC4.y) annotation (Line(points={{-33,-79},{-26.5,-79},{-26.5,-81},{-20.5,-81}}, color={0,0,127}));
      connect(EnergyGainsC3.y, EnergyGainsC2.u) annotation (Line(points={{-62.5,-59},{-68,-59},{-68,-71},{-71,-71}}, color={0,0,127}));
      connect(EnergyGainsC3.u1, heatGainsHC) annotation (Line(points={{-51,-56},{6,-56},{6,-76},{110,-76}}, color={0,0,127}));
      connect(EnergyGainsC4.u2, heatGainsHC) annotation (Line(points={{-9,-84},{18,-84},{18,-86},{70,-86},{70,-76},{110,-76}}, color={0,0,127}));
      connect(realExpression.y, EnergyGainsC3.u2) annotation (Line(points={{-7,-46},{-2,-46},{-2,-62},{-51,-62}}, color={0,0,127}));
      connect(realExpression.y, EnergyGainsC4.u1) annotation (Line(points={{-7,-46},{-2,-46},{-2,-78},{-9,-78}}, color={0,0,127}));
       annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
                -100},{100,100}}), graphics={
            Line(points={{-60,100},{0,58},{0,56},{0,40},{0,8}},       color={28,108,200}),
            Line(points={{60,100},{0,58},{-12,100},{-12,96}},   color={28,108,200}),
            Line(points={{0,42},{0,36},{0,22},{86,22},{86,-76},{100,-76}},       color={28,108,200})}),
                                    Documentation(revisions="<html>
<ul>
<li>
August 30, 2018 by Damien Picard:<br/>
Added constant StateSelectTVol to be able to select preferred state
of mixing volume.
See <a href=\"https://github.com/open-ideas/IDEAS/issues/905\">#905</a>.
</li>
<li>
July 27, 2018 by Filip Jorissen:<br/>
Added nominal values for <code>m</code>, <code>mXi</code> and <code>mC</code>.
See <a href=\"https://github.com/open-ideas/IDEAS/issues/864\">#864</a>.
</li>
<li>
Added output for the CO2 concentration.
See <a href=\"https://github.com/open-ideas/IDEAS/issues/868\">#868</a>.
</li>
<li>
April 27, 2018 by Filip Jorissen:<br/>
Created <code>MixingVolumeNominalU</code> such that 
<code>MixingVolume</code> can be used without generating a warning.
</li>
<li>
April 27, 2018 by Filip Jorissen:<br/>
Added nominal value for internal energy of mixing volume.
See <a href=\"https://github.com/open-ideas/IDEAS/issues/797\">#797</a>.
</li>
<li>
Modified model for supporting new interzonal air flow models.
Air leakage model and its parameters have been removed.
See <a href=\"https://github.com/open-ideas/IDEAS/issues/796\">#796</a>.
</li>
<li>
August 5, 2017 by Filip Jorissen:<br/>
Added support for dry air.
</li>
<li>
November 15, 2016 by Filip Jorissen:<br/>
Revised documentation.
</li>
<li>
August 26, 2016 by Filip Jorissen:<br/>
Added support for conservation of energy.
</li>
<li>
April 30, 2016, by Filip Jorissen:<br/>
First implementation.
</li>
</ul>
</html>",     info="<html>
<p>
Perfectly mixed air model.
</p>
<h4>Main equations</h4>
<p>
This model computes a single air temperature that is used to 
evaluate convective heat transfer of all surfaces,
components connected to gainCon (e.g. radiators), etc.
The air outlet temperature equals the well mixed air temperature.
</p>
<h4>Assumption and limitations</h4>
<p>
This model is not valid for buildings where stratification occurs, 
e.g. when using floor cooling
or ceiling heating.
</p>
<p>
When dry air is used, then the relative humidity output is set to zero.
</p>
<h4>Typical use and important parameters</h4>
<p>
The zone air volume <code>Vto</code> determines the thermal mass of the air.
This mass may be artificially increased using <code>mSenFac</code> if desired, 
e.g. to take into account the thermal mass of furniture.
</p>
<h4>Dynamics</h4>
This model only contains states to represent the energy and mass dynamics, 
typically using a temperature and pressure variable.
Parameters <code>energyDynamics</code> and <code>massDynamics</code>
may be used to change the model dynamics.
<h4>Validation</h4>
<p>
See BESTEST.
</p>
</html>"));
    end EvilWellMixedAir;

    model EvilHeating
      parameter Modelica.SIunits.Temp_C setTHeat(start=20)=21  "Heating temperature setpoint" annotation(Evaluate=false);
      parameter Modelica.SIunits.Temp_C setCoolDiff(start=6)=5  "Cooling temperature setpoint = setHeat-setCoolDiff" annotation(Evaluate=false);

      parameter Integer nZones=1 "Numer of zones connected to the Thermodinamic Evil";
      parameter Integer timeI=60 "Time constant for integrator - PID";
      parameter Integer timeD=60 "Time constant for derivative - PID";
      parameter Integer gainPID=1000 "Gain - PID";
      parameter Integer ymaxPIDH=1000000 "Max power heating (positive)";
      parameter Integer ymaxPIDC=ymaxPIDH "Max power cooling (positive)";

    protected
      IDEAS.Controls.Continuous.LimPID heatPID[nZones](
        each k=gainPID,
        each Ti=timeI,
        each Td=timeD,
        each yMin=0,
        each yMax=ymaxPIDH,
        each controllerType=Modelica.Blocks.Types.SimpleController.PID,
        each initType=Modelica.Blocks.Types.InitPID.InitialOutput,
        each y_start=0) annotation (Placement(transformation(extent={{0,22},{-20,42}})));
      Modelica.Thermal.HeatTransfer.Sources.PrescribedHeatFlow Heating[nZones] annotation (Placement(transformation(extent={{-30,22},{-50,42}})));
      Modelica.Blocks.Interfaces.RealInput Tsensor[nZones] annotation (Placement(transformation(extent={{-128,-76},{-88,-36}}),iconTransformation(extent={{-128,-76},{-88,-36}})));
      Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a port_a[nZones] annotation (Placement(transformation(extent={{-128,22},{-108,42}}),  iconTransformation(extent={{-128,22},{-108,42}})));
      Modelica.Thermal.HeatTransfer.Components.ThermalCollector thermalCollector[nZones](each m=2)
        annotation (Placement(transformation(
            extent={{10,-10},{-10,10}},
            rotation=270,
            origin={-78,32})));
      Modelica.Thermal.HeatTransfer.Sources.PrescribedHeatFlow Cooling[nZones] annotation (Placement(transformation(extent={{-30,-40},{-50,-20}})));
      IDEAS.Controls.Continuous.LimPID coolPID[nZones](
        each k=gainPID,
        each Ti=timeI,
        each Td=timeD,
        each yMax=0,
        each yMin=-ymaxPIDC,
        each controllerType=Modelica.Blocks.Types.SimpleController.PID,
        each initType=Modelica.Blocks.Types.InitPID.InitialOutput,
        each y_start=0) annotation (Placement(transformation(extent={{0,-20},{-20,-40}})));
      Modelica.Blocks.Sources.RealExpression setTHeating[nZones](each y=273.15 + setTHeat) "Tset Heating" annotation (Placement(transformation(extent={{68,22},{16,42}})));
      Modelica.Blocks.Sources.RealExpression setTcooling[nZones](each y=273.15 + setTHeat + setCoolDiff) "Tset Heating" annotation (Placement(transformation(extent={{68,-40},{16,-20}})));
      Modelica.Blocks.Continuous.Integrator heatingEnegy[nZones](each k=1/3600000, each y_start=0) annotation (Placement(transformation(extent={{6,70},{26,90}})));
      Modelica.Blocks.Continuous.Integrator coolingEnergy[nZones](each k=1/3600000, each y_start=0) annotation (Placement(transformation(extent={{0,-80},{20,-60}})));
      Modelica.Blocks.Math.Gain gain[nZones](each k=1/1000) "Tset Heating" annotation (Placement(transformation(extent={{34,-52},{44,-42}})));
      Modelica.Blocks.Math.Gain gain1[nZones](each k=1/1000) "Tset Heating" annotation (Placement(transformation(extent={{36,48},{46,58}})));
    public
      Modelica.Blocks.Interfaces.RealOutput HeatingEnergy[nZones] "in kWh" annotation (Placement(transformation(extent={{100,50},{140,90}})));
      Modelica.Blocks.Interfaces.RealOutput HeatingPower[nZones] "in kW" annotation (Placement(transformation(extent={{100,10},{140,50}})));
      Modelica.Blocks.Interfaces.RealOutput CoolingPower[nZones] "in kW" annotation (Placement(transformation(extent={{100,-50},{140,-10}})));
      Modelica.Blocks.Interfaces.RealOutput CoolingEnergy[nZones] "in kWh" annotation (Placement(transformation(extent={{100,-90},{140,-50}})));
      Modelica.Blocks.Interfaces.RealOutput TotalPowerH "in kW" annotation (Placement(transformation(
            extent={{-20,-20},{20,20}},
            rotation=270,
            origin={-90,-120})));
      Modelica.Blocks.Interfaces.RealOutput TotalEnergyH "in kWh" annotation (Placement(transformation(
            extent={{-20,-20},{20,20}},
            rotation=270,
            origin={-50,-120})));
      Modelica.Blocks.Interfaces.RealOutput TotalPowerC "in kW" annotation (Placement(transformation(
            extent={{-20,-20},{20,20}},
            rotation=270,
            origin={-10,-120})));
      Modelica.Blocks.Interfaces.RealOutput TotalEnergyC "in kWh" annotation (Placement(transformation(
            extent={{-20,-20},{20,20}},
            rotation=270,
            origin={30,-120})));
    equation
      TotalEnergyH=sum(heatingEnegy.y);
      TotalPowerH=sum(gain1.y);
      TotalEnergyC=sum(coolingEnergy.y);
      TotalPowerC=sum(gain.y);
      connect(setTHeating.y, heatPID.u_s) annotation (Line(points={{13.4,32},{2,32}}, color={0,0,127}));
      connect(setTcooling.y, coolPID.u_s) annotation (Line(points={{13.4,-30},{2,-30}}, color={0,0,127}));
      connect(Tsensor, heatPID.u_m) annotation (Line(points={{-108,-56},{-10,-56},{-10,20}},
                                                                                         color={0,0,127}));
      connect(Tsensor, coolPID.u_m) annotation (Line(points={{-108,-56},{-10,-56},{-10,-18}},
                                                                                          color={0,0,127}));
      connect(coolPID.y, Cooling.Q_flow) annotation (Line(points={{-21,-30},{-30,-30}}, color={0,0,127}));
      connect(thermalCollector.port_b, port_a) annotation (Line(points={{-88,32},{-118,32}},                       color={191,0,0}));
      connect(coolingEnergy.u, Cooling.Q_flow) annotation (Line(points={{-2,-70},{-30,-70},{-30,-30}}, color={0,0,127}));
      connect(heatPID.y, Heating.Q_flow) annotation (Line(points={{-21,32},{-30,32}}, color={0,0,127}));
      connect(gain.u, Cooling.Q_flow) annotation (Line(points={{33,-47},{-30,-47},{-30,-30}}, color={0,0,127}));
      connect(gain1.u, Heating.Q_flow) annotation (Line(points={{35,53},{-30,53},{-30,32}}, color={0,0,127}));
      connect(heatingEnegy.u, Heating.Q_flow) annotation (Line(points={{4,80},{-30,80},{-30,32}},  color={0,0,127}));
      connect(coolingEnergy.y, CoolingEnergy) annotation (Line(points={{21,-70},{120,-70}}, color={0,0,127}));
      connect(gain.y, CoolingPower) annotation (Line(points={{44.5,-47},{92,-47},{92,-30},{120,-30}}, color={0,0,127}));
      connect(gain1.y, HeatingPower) annotation (Line(points={{46.5,53},{80,53},{80,30},{120,30}}, color={0,0,127}));
      connect(heatingEnegy.y, HeatingEnergy) annotation (Line(points={{27,80},{70,80},{70,70},{120,70}},
                                                                                         color={0,0,127}));
      connect(Heating.port, thermalCollector.port_a[1]) annotation (Line(points={{-50,32},{-67.5,32}},                     color={191,0,0}));
      connect(Cooling.port, thermalCollector.port_a[2]) annotation (Line(points={{-50,-30},{-56,-30},{-56,32},{-68.5,32}},   color={191,0,0}));

      annotation (
        Icon(coordinateSystem(preserveAspectRatio=false), graphics={
            Ellipse(
              extent={{-72,56},{-36,18}},
              lineColor={238,46,47},
              fillColor={238,46,47},
              fillPattern=FillPattern.Solid),
            Polygon(
              points={{-70,46},{-74,52},{-74,72},{-72,78},{-70,64},{-68,56},{-62,54},{-70,46}},
              lineColor={238,46,47},
              fillColor={238,46,47},
              fillPattern=FillPattern.Solid),
            Polygon(
              points={{-40,46},{-36,52},{-36,72},{-38,78},{-40,64},{-42,56},{-48,54},{-40,46}},
              lineColor={238,46,47},
              fillColor={238,46,47},
              fillPattern=FillPattern.Solid),
            Polygon(
              points={{40,42},{44,48},{44,68},{42,74},{40,60},{38,52},{32,50},{40,42}},
              lineColor={238,46,47},
              fillColor={238,46,47},
              fillPattern=FillPattern.Solid),
            Polygon(
              points={{12,42},{8,48},{8,68},{10,74},{12,60},{14,52},{20,50},{12,42}},
              lineColor={238,46,47},
              fillColor={238,46,47},
              fillPattern=FillPattern.Solid),
            Polygon(
              points={{26,22},{26,50},{28,68},{26,74},{24,60},{22,52},{12,42},{26,22}},
              lineColor={238,46,47},
              fillColor={238,46,47},
              fillPattern=FillPattern.Solid),
            Polygon(
              points={{40,42},{26,22},{24,68},{26,74},{28,60},{30,52},{36,50},{40,42}},
              lineColor={238,46,47},
              fillColor={238,46,47},
              fillPattern=FillPattern.Solid),
            Rectangle(
              extent={{30,32},{22,-84}},
              lineColor={238,46,47},
              fillColor={238,46,47},
              fillPattern=FillPattern.Solid),
            Polygon(points={{-48,20},{-48,20}}, lineColor={28,108,200}),
            Polygon(
              points={{-52,18},{-52,10},{-40,10},{-8,-6},{20,6},{20,12},{22,12},{26,8},{22,4},{32,6},{32,-2},{18,-4},{-4,-16},{-38,-8},{-34,-42},{-38,-88},{-20,-86},{-18,-92},{-48,-96},{-44,-64},{-44,-46},
                  {-56,-48},{-56,-100},{-74,-102},{-76,-96},{-64,-94},{-64,-40},{-66,-10},{-88,-10},{-104,10},{-110,16},{-116,22},{-108,18},{-108,20},{-114,26},{-106,20},{-110,30},{-104,22},{-102,20},{-104,
                  30},{-98,20},{-96,18},{-94,26},{-94,18},{-96,12},{-82,-2},{-68,8},{-58,10},{-56,18},{-52,18}},
              fillColor={238,46,47},
              fillPattern=FillPattern.Solid,
              pattern=LinePattern.None),
            Text(
              extent={{-94,104},{-36,78}},
              lineColor={0,140,72},
              textString="%setTHeat%"),
            Text(
              extent={{-2,104},{84,78}},
              lineColor={28,108,200},
              textString="∆T=%setCoolDiff"),
            Text(
              extent={{26,76},{98,62}},
              lineColor={238,46,47},
              textString="Henergy",
              horizontalAlignment=TextAlignment.Right),
            Text(
              extent={{26,36},{98,22}},
              lineColor={238,46,47},
              horizontalAlignment=TextAlignment.Right,
              textString="Hpower"),
            Text(
              extent={{26,-22},{98,-36}},
              lineColor={28,108,200},
              horizontalAlignment=TextAlignment.Right,
              textString="Cpower"),
            Text(
              extent={{26,-62},{98,-76}},
              lineColor={28,108,200},
              horizontalAlignment=TextAlignment.Right,
              textString="Cenergy"),
            Text(
              extent={{-106,-140},{-74,-154}},
              lineColor={238,46,47},
              textString="ΣHP"),
            Text(
              extent={{-66,-140},{-34,-154}},
              lineColor={238,46,47},
              textString="ΣHE"),
            Text(
              extent={{-26,-140},{6,-154}},
              lineColor={28,108,200},
              textString="ΣCP"),
            Text(
              extent={{14,-140},{46,-154}},
              lineColor={28,108,200},
              textString="ΣCE")}),
        Diagram(coordinateSystem(preserveAspectRatio=false)));
    end EvilHeating;

    model EvilVentilation
       parameter Integer nZones=1;
       parameter Integer globalFlow=1000;
       parameter Integer EvilMassFlowRate[nZones]=ones(nZones)*globalFlow;
       parameter Modelica.SIunits.Temp_C setTemp = 20 "Temperature setpoint";
       parameter Modelica.SIunits.Temp_C arrayT[nZones]=ones(nZones)*(273.15 + setTemp)*2;


      Modelica.Blocks.Sources.RealExpression EvilFlow[nZones](y=EvilMassFlowRate)
                                                                           annotation (Placement(transformation(extent={{40,46},{12,66}})));
      IDEAS.Fluid.Movers.FlowControlled_m_flow fan[nZones](
        redeclare each package Medium = IDEAS.Media.Air,
        each m_flow_nominal=1,
        redeclare each IDEAS.Fluid.Movers.Data.Pumps.Wilo.CronolineIL80slash220dash4slash4 per,
        each addPowerToMedium=false,
        each nominalValuesDefineDefaultPressureCurve=true,
        each dp_nominal=0) annotation (Placement(transformation(extent={{16,20},{-4,40}})));
      IDEAS.Fluid.Sensors.EnthalpyFlowRate senEntFloEN[nZones](redeclare package Medium = IDEAS.Media.Air, each m_flow_nominal=1) annotation (Placement(transformation(extent={{-30,24},{-42,36}})));
      IDEAS.Fluid.Sensors.EnthalpyFlowRate senEntFlo[nZones](redeclare package Medium = IDEAS.Media.Air, each m_flow_nominal=1) annotation (Placement(transformation(extent={{-44,-6},{-32,6}})));
      IDEAS.Fluid.Sources.Boundary_pT bou[nZones](
        redeclare package Medium = IDEAS.Media.Air,
        each use_T_in=true,
        nPorts=2) annotation (Placement(transformation(extent={{56,16},{36,36}})));
      Modelica.Blocks.Interfaces.RealOutput Power[nZones] annotation (Placement(transformation(extent={{100,-22},{120,-2}})));
      Modelica.Blocks.Interfaces.RealOutput EnergyHeating[nZones] annotation (Placement(transformation(extent={{98,-90},{118,-70}})));
      Modelica.Blocks.Math.Max max[nZones] "Heat gains in kWh" annotation (Placement(transformation(
            extent={{-5,-5},{5,5}},
            rotation=0,
            origin={33,-55})));
      Modelica.Blocks.Continuous.Integrator EnergyGainsCooling[nZones](each k=1/3600) "Heat gains in kWh"
        annotation (Placement(transformation(
            extent={{-5,-5},{5,5}},
            rotation=0,
            origin={55,-21})));
      Modelica.Blocks.Math.Min min[nZones] "Heat gains in kWh" annotation (Placement(transformation(
            extent={{-5,-5},{5,5}},
            rotation=0,
            origin={33,-21})));
      Modelica.Blocks.Continuous.Integrator EnergyGainsH[nZones](each k=1/3600) "Heat gains in kWh"
        annotation (Placement(transformation(
            extent={{-5,-5},{5,5}},
            rotation=0,
            origin={63,-79})));
      Modelica.Blocks.Sources.RealExpression realExpression[nZones] annotation (Placement(transformation(extent={{52,-50},{32,-30}})));
      Modelica.Blocks.Interfaces.RealOutput EnergyCooling[nZones] annotation (Placement(transformation(extent={{100,-60},{120,-40}})));
      Modelica.Blocks.Math.Add add[nZones](each k2=-1) "Heat gains in kWh" annotation (Placement(transformation(
            extent={{-5,-5},{5,5}},
            rotation=0,
            origin={7,-17})));
      Modelica.Blocks.Interfaces.RealOutput TPC annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=270,
            origin={-76,-110})));
      Modelica.Blocks.Interfaces.RealOutput TPH annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=270,
            origin={-48,-110})));
      Modelica.Blocks.Interfaces.RealOutput TEnC annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=270,
            origin={-16,-110})));
      Modelica.Blocks.Interfaces.RealOutput TEnH annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=270,
            origin={10,-110})));
      Modelica.Blocks.Sources.RealExpression EvilFlow1
                                                     [nZones](y=300)       annotation (Placement(transformation(extent={{52,46},{76,64}})));

    equation
      Power=add.y/1000;
      TEnC=sum(EnergyCooling);
      TEnH=sum(EnergyHeating);
      TPC=sum(min.y)/3600;
      TPH=sum(max.y)/3600;

      connect(fan.m_flow_in, EvilFlow.y) annotation (Line(points={{6,42},{6,56},{10.6,56}},
                                                                                          color={0,0,127}));
      connect(fan.port_a, bou.ports[1]) annotation (Line(points={{16,30},{36,30},{36,28}}, color={0,127,255}));
      connect(senEntFlo.port_b, bou.ports[2]) annotation (Line(points={{-32,0},{28,0},{28,24},{36,24}}, color={0,127,255}));
      connect(fan.port_b, senEntFloEN.port_a) annotation (Line(points={{-4,30},{-30,30}}, color={0,127,255}));
      connect(senEntFloEN.H_flow, add.u1) annotation (Line(points={{-36,36.6},{-36,46},{-18,46},{-18,-14},{1,-14}}, color={0,0,127}));
      connect(senEntFlo.H_flow, add.u2) annotation (Line(points={{-38,6.6},{-38,14},{-22,14},{-22,-20},{1,-20}}, color={0,0,127}));
      connect(add.y, min.u1) annotation (Line(points={{12.5,-17},{18.25,-17},{18.25,-18},{27,-18}}, color={0,0,127}));
      connect(min.u2, realExpression.y) annotation (Line(points={{27,-24},{22,-24},{22,-40},{31,-40}}, color={0,0,127}));
      connect(realExpression.y,max. u1) annotation (Line(points={{31,-40},{22,-40},{22,-52},{27,-52}}, color={0,0,127}));
      connect(add.y,max. u2) annotation (Line(points={{12.5,-17},{14,-17},{14,-58},{27,-58}}, color={0,0,127}));
      connect(max.y, EnergyGainsH.u) annotation (Line(points={{38.5,-55},{46,-55},{46,-80},{57,-80},{57,-79}}, color={0,0,127}));
      connect(EnergyGainsH.y, EnergyHeating) annotation (Line(points={{68.5,-79},{108,-79},{108,-80}}, color={0,0,127}));
      connect(EnergyGainsCooling.y, EnergyCooling) annotation (Line(points={{60.5,-21},{72,-21},{72,-50},{110,-50}}, color={0,0,127}));
      connect(min.y, EnergyGainsCooling.u) annotation (Line(points={{38.5,-21},{44.25,-21},{44.25,-21},{49,-21}}, color={0,0,127}));
      connect(TPH, TPH) annotation (Line(points={{-48,-110},{-48,-110}}, color={0,0,127}));
      connect(EvilFlow1.y, bou.T_in) annotation (Line(points={{77.2,55},{77.2,30},{58,30}}, color={0,0,127}));
      connect(senEntFloEN.port_b, senEntFlo.port_a) annotation (Line(points={{-42,30},{-52,30},{-52,32},{-58,32},{-58,0},{-44,0}}, color={0,127,255}));
      annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
            Polygon(
              points={{26,22},{26,50},{28,68},{26,74},{24,60},{22,52},{12,42},{26,22}},
              fillColor={28,108,200},
              fillPattern=FillPattern.Solid,
              pattern=LinePattern.None,
              lineColor={0,0,0}),
            Polygon(
              points={{12,42},{8,48},{8,68},{10,74},{12,60},{14,52},{20,50},{12,42}},
              fillColor={28,108,200},
              fillPattern=FillPattern.Solid,
              pattern=LinePattern.None,
              lineColor={0,0,0}),
            Polygon(
              points={{40,42},{44,48},{44,68},{42,74},{40,60},{38,52},{32,50},{40,42}},
              fillColor={28,108,200},
              fillPattern=FillPattern.Solid,
              pattern=LinePattern.None,
              lineColor={0,0,0}),
            Text(
              extent={{28,-4},{100,-18}},
              horizontalAlignment=TextAlignment.Right,
              pattern=LinePattern.None,
              lineColor={0,0,0},
              fillColor={28,108,200},
              fillPattern=FillPattern.Solid,
              textString="Gains Power"),
            Polygon(
              points={{40,42},{26,22},{24,68},{26,74},{28,60},{30,52},{36,50},{40,42}},
              fillColor={28,108,200},
              fillPattern=FillPattern.Solid,
              pattern=LinePattern.None,
              lineColor={0,0,0}),
            Polygon(
              points={{-52,18},{-52,10},{-40,10},{-8,-6},{20,6},{20,12},{22,12},{26,8},{22,4},{32,6},{32,-2},{18,-4},{-4,-16},{-38,-8},{-34,-42},{-38,-88},{-20,-86},{-18,-92},{-48,-96},{-44,-64},{-44,-46},
                  {-56,-48},{-56,-100},{-74,-102},{-76,-96},{-64,-94},{-64,-40},{-66,-10},{-88,-10},{-104,10},{-110,16},{-116,22},{-108,18},{-108,20},{-114,26},{-106,20},{-110,30},{-104,22},{-102,20},{-104,
                  30},{-98,20},{-96,18},{-94,26},{-94,18},{-96,12},{-82,-2},{-68,8},{-58,10},{-56,18},{-52,18}},
              fillColor={28,108,200},
              fillPattern=FillPattern.Solid,
              pattern=LinePattern.None,
              lineColor={0,0,0}),
            Ellipse(
              extent={{-72,56},{-36,18}},
              fillColor={28,108,200},
              fillPattern=FillPattern.Solid,
              pattern=LinePattern.None,
              lineColor={0,0,0}),
            Polygon(
              points={{-40,46},{-36,52},{-36,72},{-38,78},{-40,64},{-42,56},{-48,54},{-40,46}},
              fillColor={28,108,200},
              fillPattern=FillPattern.Solid,
              pattern=LinePattern.None,
              lineColor={0,0,0}),
            Polygon(
              points={{-70,46},{-74,52},{-74,72},{-72,78},{-70,64},{-68,56},{-62,54},{-70,46}},
              fillColor={28,108,200},
              fillPattern=FillPattern.Solid,
              pattern=LinePattern.None,
              lineColor={0,0,0}),
            Rectangle(
              extent={{30,32},{22,-84}},
              fillColor={28,108,200},
              fillPattern=FillPattern.Solid,
              pattern=LinePattern.None,
              lineColor={0,0,0}),
            Text(
              extent={{28,-42},{100,-56}},
              horizontalAlignment=TextAlignment.Right,
              pattern=LinePattern.None,
              lineColor={0,0,0},
              fillColor={28,108,200},
              fillPattern=FillPattern.Solid,
              textString="Energy lost"),
            Text(
              extent={{26,-72},{98,-86}},
              horizontalAlignment=TextAlignment.Right,
              pattern=LinePattern.None,
              lineColor={0,0,0},
              fillColor={28,108,200},
              fillPattern=FillPattern.Solid,
              textString="Energy gain")}),                           Diagram(coordinateSystem(preserveAspectRatio=false)));
    end EvilVentilation;

    model EvilHeatingSimpel
      parameter Modelica.SIunits.Temp_C setTHeat=20  "Heating temperature setpoint" annotation(Evaluate=false);
      parameter Modelica.SIunits.Temp_C setCoolDiff=6  "Cooling temperature setpoint = setHeat-setCoolDiff" annotation(Evaluate=false);

      parameter Integer nZones=1 "Numer of zones connected to the Thermodinamic Evil";
      parameter Integer timeI=60 "Time constant for integrator - PID";
      parameter Integer timeD=60 "Time constant for derivative - PID";
      parameter Integer gainPID=1000 "Gain - PID";
      parameter Integer ymaxPIDH=1000000 "Max power heating (positive)";
      parameter Integer ymaxPIDC=ymaxPIDH "Max power cooling (positive)";

      IDEAS.Controls.Continuous.LimPID heatPID[nZones](
        each k=gainPID,
        each Ti=timeI,
        each Td=timeD,
        each yMin=0,
        each yMax=ymaxPIDH,
        each controllerType=Modelica.Blocks.Types.SimpleController.PID,
        each initType=Modelica.Blocks.Types.InitPID.InitialOutput,
        each y_start=0) annotation (Placement(transformation(extent={{24,22},{4,42}})));
      Modelica.Thermal.HeatTransfer.Sources.PrescribedHeatFlow Heating[nZones] annotation (Placement(transformation(extent={{-24,22},{-44,42}})));
      Modelica.Blocks.Interfaces.RealInput Tsensor[nZones] annotation (Placement(transformation(extent={{-128,-76},{-88,-36}}),iconTransformation(extent={{-128,-76},{-88,-36}})));
      Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a port_a[nZones] annotation (Placement(transformation(extent={{-128,22},{-108,42}}),  iconTransformation(extent={{-128,22},{-108,42}})));
      Modelica.Thermal.HeatTransfer.Components.ThermalCollector thermalCollector[nZones](each m=2)
        annotation (Placement(transformation(
            extent={{10,-10},{-10,10}},
            rotation=270,
            origin={-78,32})));
      Modelica.Thermal.HeatTransfer.Sources.PrescribedHeatFlow Cooling[nZones] annotation (Placement(transformation(extent={{-24,-40},{-44,-20}})));
      IDEAS.Controls.Continuous.LimPID coolPID[nZones](
        each k=gainPID,
        each Ti=timeI,
        each Td=timeD,
        each yMax=0,
        each yMin=-ymaxPIDC,
        each controllerType=Modelica.Blocks.Types.SimpleController.PID,
        each initType=Modelica.Blocks.Types.InitPID.InitialOutput,
        each y_start=0) annotation (Placement(transformation(extent={{24,-20},{4,-40}})));
      Modelica.Blocks.Sources.RealExpression setTHeating[nZones](each y=273.15 + setTHeat) "Tset Heating" annotation (Placement(transformation(extent={{92,22},{40,42}})));
      Modelica.Blocks.Sources.RealExpression setTcooling[nZones](each y=273.15 + setTHeat + setCoolDiff) "Tset Heating" annotation (Placement(transformation(extent={{92,-40},{40,-20}})));
    equation
      connect(setTHeating.y, heatPID.u_s) annotation (Line(points={{37.4,32},{26,32}},color={0,0,127}));
      connect(setTcooling.y, coolPID.u_s) annotation (Line(points={{37.4,-30},{26,-30}},color={0,0,127}));
      connect(Tsensor, coolPID.u_m) annotation (Line(points={{-108,-56},{-70,-56},{-70,6},{14,6},{14,-18}},
                                                                                          color={0,0,127}));
      connect(coolPID.y, Cooling.Q_flow) annotation (Line(points={{3,-30},{-24,-30}},   color={0,0,127}));
      connect(thermalCollector.port_b, port_a) annotation (Line(points={{-88,32},{-118,32}},                       color={191,0,0}));
      connect(heatPID.y, Heating.Q_flow) annotation (Line(points={{3,32},{-24,32}},   color={0,0,127}));
      connect(Heating.port, thermalCollector.port_a[1]) annotation (Line(points={{-44,32},{-67.5,32}},                     color={191,0,0}));
      connect(Cooling.port, thermalCollector.port_a[2]) annotation (Line(points={{-44,-30},{-56,-30},{-56,32},{-68.5,32}},   color={191,0,0}));

      connect(Tsensor, heatPID.u_m) annotation (Line(points={{-108,-56},{-70,-56},{-70,6},{14,6},{14,20}}, color={0,0,127}));
      annotation (
        Icon(coordinateSystem(preserveAspectRatio=false), graphics={
            Ellipse(
              extent={{-72,56},{-36,18}},
              lineColor={238,46,47},
              fillColor={238,46,47},
              fillPattern=FillPattern.Solid),
            Polygon(
              points={{-70,46},{-74,52},{-74,72},{-72,78},{-70,64},{-68,56},{-62,54},{-70,46}},
              lineColor={238,46,47},
              fillColor={238,46,47},
              fillPattern=FillPattern.Solid),
            Polygon(
              points={{-40,46},{-36,52},{-36,72},{-38,78},{-40,64},{-42,56},{-48,54},{-40,46}},
              lineColor={238,46,47},
              fillColor={238,46,47},
              fillPattern=FillPattern.Solid),
            Polygon(
              points={{40,42},{44,48},{44,68},{42,74},{40,60},{38,52},{32,50},{40,42}},
              lineColor={238,46,47},
              fillColor={238,46,47},
              fillPattern=FillPattern.Solid),
            Polygon(
              points={{12,42},{8,48},{8,68},{10,74},{12,60},{14,52},{20,50},{12,42}},
              lineColor={238,46,47},
              fillColor={238,46,47},
              fillPattern=FillPattern.Solid),
            Polygon(
              points={{26,22},{26,50},{28,68},{26,74},{24,60},{22,52},{12,42},{26,22}},
              lineColor={238,46,47},
              fillColor={238,46,47},
              fillPattern=FillPattern.Solid),
            Polygon(
              points={{40,42},{26,22},{24,68},{26,74},{28,60},{30,52},{36,50},{40,42}},
              lineColor={238,46,47},
              fillColor={238,46,47},
              fillPattern=FillPattern.Solid),
            Rectangle(
              extent={{30,32},{22,-84}},
              lineColor={238,46,47},
              fillColor={238,46,47},
              fillPattern=FillPattern.Solid),
            Polygon(points={{-48,20},{-48,20}}, lineColor={28,108,200}),
            Polygon(
              points={{-52,18},{-52,10},{-40,10},{-8,-6},{20,6},{20,12},{22,12},{26,8},{22,4},{32,6},{32,-2},{18,-4},{-4,-16},{-38,-8},{-34,-42},{-38,-88},{-20,-86},{-18,-92},{-48,-96},{-44,-64},{-44,-46},
                  {-56,-48},{-56,-100},{-74,-102},{-76,-96},{-64,-94},{-64,-40},{-66,-10},{-88,-10},{-104,10},{-110,16},{-116,22},{-108,18},{-108,20},{-114,26},{-106,20},{-110,30},{-104,22},{-102,20},{-104,
                  30},{-98,20},{-96,18},{-94,26},{-94,18},{-96,12},{-82,-2},{-68,8},{-58,10},{-56,18},{-52,18}},
              fillColor={238,46,47},
              fillPattern=FillPattern.Solid,
              pattern=LinePattern.None),
            Text(
              extent={{-94,104},{-36,78}},
              lineColor={0,140,72},
              textString="%setTHeat%"),
            Text(
              extent={{-2,104},{84,78}},
              lineColor={28,108,200},
              textString="∆T=%setCoolDiff")}),
        Diagram(coordinateSystem(preserveAspectRatio=false)));
    end EvilHeatingSimpel;

    model ProtectedTest
        parameter Modelica.SIunits.Temp_C prot1(start=20)=21  "protected component" annotation(Evaluate=false);
        parameter Modelica.SIunits.Temp_C prot2(start=6)=5  "protected component" annotation(Evaluate=false);
      annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
            Ellipse(
              extent={{-100,100},{100,-100}},
              lineColor={28,108,200},
              fillColor={255,255,0},
              fillPattern=FillPattern.Solid),
            Polygon(
              points={{0,100},{-10,100},{-22,98},{-40,92},{-56,84},{-66,76},{-84,56},{-92,40},{-96,30},{-98,24},{-100,12},{-100,0},{0,0},{0,100}},
              lineColor={28,108,200},
              fillColor={0,0,0},
              fillPattern=FillPattern.Solid),
            Polygon(
              points={{50,50},{40,50},{28,48},{10,42},{-6,34},{-16,26},{-34,6},{-42,-10},{-46,-20},{-48,-26},{-50,-38},{-50,-50},{50,-50},{50,50}},
              lineColor={28,108,200},
              fillColor={0,0,0},
              fillPattern=FillPattern.Solid,
              origin={50,-50},
              rotation=180)}),                                       Diagram(coordinateSystem(preserveAspectRatio=false)));
    end ProtectedTest;
  end Components;

  annotation (uses(IDEAS(version="2.0.0"), Modelica(version="3.2.2")), Icon(graphics={
        Polygon(
          points={{-12,34},{-12,26},{0,26},{32,10},{60,22},{60,28},{62,28},{66,24},{62,20},{72,22},{72,14},{58,12},{36,0},{2,8},{6,-26},{2,-72},{20,-70},{22,-76},{-8,-80},{-4,-48},{-4,-30},{-16,-32},
              {-16,-84},{-34,-86},{-36,-80},{-24,-78},{-24,-24},{-26,6},{-48,6},{-64,26},{-70,32},{-76,38},{-68,34},{-68,36},{-74,42},{-66,36},{-70,46},{-64,38},{-62,36},{-64,46},{-58,36},{-56,34},{
              -54,42},{-54,34},{-56,28},{-42,14},{-28,24},{-18,26},{-16,34},{-12,34}},
          fillColor={238,46,47},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None),
        Ellipse(
          extent={{-32,72},{4,34}},
          lineColor={238,46,47},
          fillColor={238,46,47},
          fillPattern=FillPattern.Solid),
        Polygon(
          points={{0,62},{4,68},{4,88},{2,94},{0,80},{-2,72},{-8,70},{0,62}},
          lineColor={238,46,47},
          fillColor={238,46,47},
          fillPattern=FillPattern.Solid),
        Polygon(
          points={{-30,62},{-34,68},{-34,88},{-32,94},{-30,80},{-28,72},{-22,70},{-30,62}},
          lineColor={238,46,47},
          fillColor={238,46,47},
          fillPattern=FillPattern.Solid),
        Polygon(
          points={{80,48},{66,28},{64,74},{66,80},{68,66},{70,58},{76,56},{80,48}},
          lineColor={238,46,47},
          fillColor={238,46,47},
          fillPattern=FillPattern.Solid),
        Polygon(
          points={{52,48},{48,54},{48,74},{50,80},{52,66},{54,58},{60,56},{52,48}},
          lineColor={238,46,47},
          fillColor={238,46,47},
          fillPattern=FillPattern.Solid),
        Polygon(
          points={{80,48},{84,54},{84,74},{82,80},{80,66},{78,58},{72,56},{80,48}},
          lineColor={238,46,47},
          fillColor={238,46,47},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{70,38},{62,-78}},
          lineColor={238,46,47},
          fillColor={238,46,47},
          fillPattern=FillPattern.Solid),
        Polygon(
          points={{52,48},{66,28},{68,74},{66,80},{64,66},{62,58},{56,56},{52,48}},
          lineColor={238,46,47},
          fillColor={238,46,47},
          fillPattern=FillPattern.Solid)}));
end Evil;
