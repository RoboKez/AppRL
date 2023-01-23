//
//  ManView.swift
//  robot
//
//  Created by Kez Smithson Whitehead on 24/05/2022.
//

import SwiftUI

struct ManView: View {
    @EnvironmentObject var rl: RL
    @StateObject var control = Control()
    var body: some View {
        
        if(rl.real && !rl.portConnected){
            serialView
//            control.m_inverter = true;
//            control.m_pitchOnly = true;
        } else {
        
            VStack{
                HStack{
                    Text("Manual Control")
                        .fontWeight(.ultraLight)
                        .foregroundColor(Color.green)
                        .multilineTextAlignment(.center)
                        .padding(.all,10)
                        .font(.title)
                        .shadow(color: .green, radius: 10)
                        .padding()
                   
                    Button(action: {
                        rl.goHome()
                    }, label: {Label("Home", systemImage: "house")
                        .foregroundColor(Color.green)})
                        .padding()
                }
                Divider()
                VStack{
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
                                DongleCommand(-0.3, 0);
                            }, label: {Label("Forward", systemImage: "tuningfork")
                                .foregroundColor(Color.green)})
                                .padding()
                            
                            Button(action:{
                                DongleCommand(0.0, 0);
                            }, label: {Label("Zero", systemImage: "tuningfork")
                                .foregroundColor(Color.green)})
                                .padding()
                            
                            Button(action:{
                                DongleCommand(0.3, 0);
                            }, label: {Label("Back", systemImage: "tuningfork")
                                .foregroundColor(Color.green)})
                                .padding()
                            
                            Button(action:{
                                DongleCommand(0, 0.3);
                            }, label: {Label("Left", systemImage: "tuningfork")
                                .foregroundColor(Color.green)})
                                .padding()
                            
                            Button(action:{
                                DongleCommand(0, -0.3);
                            }, label: {Label("Right", systemImage: "tuningfork")
                                .foregroundColor(Color.green)})
                                .padding()
                            
                            Button(action:{
                                DongleBegin(rl.curPort)
//                                setDriveMode(1)
//                                manReset()
//                                setSetPoint(0)
//                                StepLimited(0)
//                                rlEpisodeReset(0) // note start position not used for real system
//
//                                let tmp_ob = getEnvOb()!
//                                var ob: [Float] = []
//                                for j in 0...getObDim()-1 {
//                                    ob.append(tmp_ob[Int(j)])
//                                }
//                                print("Reset ob ", ob)
//                                var done = false
//                                ////////////////////////
//                                control.Reset()
//                                while(!done){
//                                    var act:[Float] = [getSetPoint(), getSetPoint2(), 0]
//
//                                    print("ob: ",ob[0])
//                                    let done_type = manStep(UnsafeMutablePointer(&act))
//                                    if (done_type == 1 || done_type == 3){
//                                        done = true
//                                    }
//                                    let tmp_ob = getEnvOb()!
//                                    ob = []
//                                    for j in 0...getObDim()-1 {
//                                        ob.append(tmp_ob[Int(j)])
//                                    }
//                                    if (!control.m_running || (checkUserQuit() != 0)){break}
//                                }
//                                StepLimited(1)
//                                rl.closeExperiment()
//                                control.m_running = false;
//                                setDriveMode(0)
                                
                            }, label: {Label("Test", systemImage: "tuningfork")
                                .foregroundColor(Color.green)})
                                .padding()
                        }
                    }
                }
            }
        }
    }
}
// Real robot serial setup ===============================================================================
extension ManView {
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

