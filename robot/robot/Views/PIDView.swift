//
//  PIDView.swift
//  robot
//
//  Created by Kez Smithson Whitehead on 24/05/2022.
//

import SwiftUI

struct PIDView: View {
    @EnvironmentObject var rl: RL
    @StateObject var control = Control()
    @State var targetVel:Float = 0
    @State var EpsReturn:Float = 0
    var body: some View {
        
        if(rl.real && !rl.portConnected){
            serialView
//            control.m_inverter = true;
//            control.m_pitchOnly = true;
        } else {
        
            VStack{
                HStack{
                    if(rl.real){
                        Text("Real PID Tuner")
                            .fontWeight(.ultraLight)
                            .foregroundColor(Color.green)
                            .multilineTextAlignment(.center)
                            .padding(.all,10)
                            .font(.title)
                            .shadow(color: .green, radius: 10)
                            .padding()
                        
                    } else {
                        Text("Simulation PID Tuner")
                            .fontWeight(.ultraLight)
                            .foregroundColor(Color.green)
                            .multilineTextAlignment(.center)
                            .padding(.all,10)
                            .font(.title)
                            .shadow(color: .green, radius: 10)
                            .padding()
                    }
                    Toggle("Inverter", isOn: $control.m_inverter).toggleStyle(.switch)
                        
                    Spacer()
                    Toggle("Pitch only", isOn: $control.m_pitchOnly).toggleStyle(.switch)
                    Spacer()
                    Text("Undiscounted Return: "+String(EpsReturn))
                    
                    Button(action: {
                        rl.goHome()
                    }, label: {Label("Home", systemImage: "house")
                        .foregroundColor(Color.green)})
                        .padding()
                }
                Divider()
                Spacer()
                if(!rl.real){
                    HStack{
                        GraphView(title: "Pitch",
                                  x1: control.xtime, y1: control.ya,
                                  x2: control.xtime, y2: control.yat,
                                  l1: "Actual", l2: "Target",
                                  x_title: "Step", y_title: "Angle",
                                  minManual: -0, maxManual: 0).padding()
    
    
                        if(!control.m_pitchOnly){
                            GraphView(title: "Wheel Velocity",
                                      x1: control.xtime, y1: control.yv,
                                      x2: control.xtime, y2: control.yvt,
                                      l1: "Actual", l2: "Target",
                                      x_title: "Step", y_title: "Velocity",
                                      minManual: -0, maxManual: 0).padding()
                        }
    
    
                    }
                    Divider()
                    HStack{
                        BarView(title: "Pitch", P:control.m_Pa, I:control.m_Ia, D:control.m_Da,
                                limP: control.m_limIa, limI: control.m_limIa, limD: control.m_limIa).padding()
    
                        if(!control.m_pitchOnly){
                            BarView(title: "Velocity", P:control.m_Pv, I:control.m_Iv, D:control.m_Dv,
                                    limP: control.m_limIv, limI: control.m_limIv, limD: control.m_limIv).padding()
                        }
                    }
                    
                }

                Divider()
                VStack{
                    HStack{
                        PitchPIDView
                        
                        if(!control.m_pitchOnly){
                            WheelPIDView
                        }
                        
                        
                    }
                    
                    HStack{
                        if control.m_running{
                            Button(action: {
                                control.m_running = false
                                setWatching(0)
                            }, label: {Label("Stop", systemImage: "stop.fill")
                                .foregroundColor(Color.red)})
                                .padding()
                            
                        } else {
                            Button(action:{
    //                            var ob = rl.pidReset()
                                
                                manReset()
                                rlEpisodeReset(2)
                                setWatching(1)
                                setFramerate(120)
                                setSetPoint(0)
                                StepLimited(0)
                                
                                
                                let tmp_ob = getEnvOb()!
                                var ob: [Float] = []
                                for j in 0...getObDim()-1 {
                                    ob.append(tmp_ob[Int(j)])
                                }
                                print("Reset ob ", ob)
                                var done = false
                                EpsReturn = 0;
                                control.Reset()
                                while(!done){
                                    var act_0:Float = 0
                                    if(control.m_pitchOnly){
                                        act_0 = control.justStabalise(tarAng: control.m_tarPitch,
                                                                          curPitch: ob[0],
                                                                           Ts: 0.025)
                                    } else {
                                        act_0 = control.Cascade(tarVel: targetVel,
                                                                    curVel: ob[2],
                                                                    curPitch: ob[0],
                                                                    Ts: 0.025)
                                    }
                                    var act:[Float] = [act_0]
                                    
//                                    print("ob: ",ob[0],"act", act_0)
                                    let done_type = manStep(UnsafeMutablePointer(&act))
//                                    if (done_type == 1 || done_type == 3){
                                    EpsReturn += getReward();
                                    if (done_type != 0){
                                        done = true
                                    }
                                    
                                    let tmp_ob = getEnvOb()!
                                    ob = []
                                    for j in 0...getObDim()-1 {
                                        ob.append(tmp_ob[Int(j)])
                                    }
                                    print(ob[0])
                                    if (!control.m_running || (checkUserQuit() != 0)){break}
                                }
                                StepLimited(1)
                                rl.closeExperiment()
//                                manualTerminate()
                                control.m_running = false;
                                
                                
                            }, label: {Label("Test", systemImage: "tuningfork")
                                .foregroundColor(Color.green)})
                                .padding()
                        }
                        
                        Button(action:{
                            control.defaultParams()
                        }, label: {Label("Default Values", systemImage: "slider.horizontal.3")})
                            .padding()
                        
                    }
                }
            }
        }
    }
}


extension PIDView {
    private var PitchPIDView: some View {

        ScrollView {
            Text("Angle Error:\t" + String(control.m_Ea))
            HStack{
                Text("Target")
                Slider(
                    value: $control.m_tarPitch,
                    in: -0.1...0.1,
                            onEditingChanged: { editing in
                                rl.isEditing = editing
                            }
                        )
                Text("\(control.m_tarPitch)")
                    .foregroundColor(rl.isEditing ? .red : .blue)
            }
            

            HStack{
                Text("KP")
                Slider(
                    value: $control.m_KPa,
                            in: 0...100,
                            onEditingChanged: { editing in
                                rl.isEditing = editing
                            }
                        )
                Text("\(control.m_KPa)")
                    .foregroundColor(rl.isEditing ? .red : .blue)
            }
            HStack{
                Text("KI")
                Slider(
                    value: $control.m_KIa,
                            in: 0...100,
                            onEditingChanged: { editing in
                                rl.isEditing = editing
                            }
                        )
                Text("\(control.m_KIa)")
                    .foregroundColor(rl.isEditing ? .red : .blue)
            }

            HStack{
                Text("KD")
                Slider(
                    value: $control.m_KDa,
                            in: 0...10,
                            onEditingChanged: { editing in
                                rl.isEditing = editing
                            }
                        )
                Text("\(control.m_KDa)")
                    .foregroundColor(rl.isEditing ? .red : .blue)
            }

            HStack{
                Text("Interger Limit")
                Slider(
                    value: $control.m_limIa,
                    in: 0...100,
                            onEditingChanged: { editing in
                                rl.isEditing = editing
                            }
                        )
                Text("\(control.m_limIa)")
                    .foregroundColor(rl.isEditing ? .red : .blue)
            }
        }.padding()
    }
}


extension PIDView {
    private var WheelPIDView: some View {

        ScrollView {
            Text("Velocity Error:\t" + String(control.m_Ev))
            HStack{
                Text("Target")
                Slider(
                    value: $targetVel,
                    in: -2.0...2.0,
                            onEditingChanged: { editing in
                                rl.isEditing = editing
                            }
                        )
                Text("\(targetVel)")
                    .foregroundColor(rl.isEditing ? .red : .blue)
            }
            

            HStack{
                Text("KP")
                Slider(
                    value: $control.m_KPv,
                            in: 0...1,
                            onEditingChanged: { editing in
                                rl.isEditing = editing
                            }
                        )
                Text("\(control.m_KPv)")
                    .foregroundColor(rl.isEditing ? .red : .blue)
            }
            HStack{
                Text("KI")
                Slider(
                    value: $control.m_KIv,
                            in: 0...1,
                            onEditingChanged: { editing in
                                rl.isEditing = editing
                            }
                        )
                Text("\(control.m_KIv)")
                    .foregroundColor(rl.isEditing ? .red : .blue)
            }

            HStack{
                Text("KD")
                Slider(
                    value: $control.m_KDv,
                            in: 0...1,
                            onEditingChanged: { editing in
                                rl.isEditing = editing
                            }
                        )
                Text("\(control.m_KDv)")
                    .foregroundColor(rl.isEditing ? .red : .blue)
            }

            HStack{
                Text("Interger Limit")
                Slider(
                    value: $control.m_limIv,
                    in: 0...100,
                            onEditingChanged: { editing in
                                rl.isEditing = editing
                            }
                        )
                Text("\(control.m_limIv)")
                    .foregroundColor(rl.isEditing ? .red : .blue)
            }
        }.padding()
    }
}


// Real robot serial setup ===============================================================================
extension PIDView {
    private var serialView: some View {
        
        HStack{
            Menu(rl.curPort) {
                Button(action: {
                    rl.updatePorts()
                }, label: {
                    Text("refresh port search").foregroundColor(Color.green)
                })
                ForEach(rl.ports, id: \.self){ p in
                    Button(p) {
                        rl.curPort = p
                    }
                }
            }.padding()
            
            if (rl.ports.count > 0 && (rl.curPort != "Select USB port...")){
                Button(action:{
                    rl.portConnected = true
                    setPort(rl.curPort)
                }, label: {
                    Label("Connect", systemImage: "cable.connector")
                })
            }
            Button(action: {
                rl.goHome()
            }, label: {Label("Home", systemImage: "house")
                .foregroundColor(Color.green)})
                .padding()
        }
    }
}
