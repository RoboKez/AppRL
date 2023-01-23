//
//  BaseView.swift
//  robot
//
//  Created by Kez Smithson Whitehead on 31/03/2022.
//

import SwiftUI

struct HomeView: View {
    @EnvironmentObject var rl: RL
    var body: some View {
            VStack{
                Text("-=AppRL_Bytes=-")
                    .fontWeight(.ultraLight)
                    .foregroundColor(Color.green)
                    .multilineTextAlignment(.center)
                    .padding(.all,10)
                    .font(.title)
                    .shadow(color: .green, radius: 10)
                    .padding()
                Text("Kez Smithson Whitehead")
                    .fontWeight(.ultraLight)
                    .foregroundColor(Color.green)
                    .multilineTextAlignment(.center)
                    .padding(.all,10)
                    .font(.title)
                    .shadow(color: .green, radius: 10)
                    .padding()
                HStack{
                    Label("Select", systemImage: "folder")
                    
                    Menu("Select your Experiment...") {
                        
                        Button(action:{
                            rl.newExp = true
                            rl.comForm[0] = true
                            rl.comNames[0] = "tmp"
                            rl.view = 2
                        }, label: {
                            Text("Create new experiment").foregroundColor(.green)
                        })
                        
                        ForEach(rl.files, id: \.self){ f in
                            Button(f) {
                                print(f)
                                let ch = Character(".")
                                let result = f.split(separator: ch).map { String($0) }
                                rl.comNames[0] = result[0]
                                rl.newExp = false
                                setPathAndFile(rl.path, rl.comNames[0]+".json", rl.comNames[0]+"-sub.json", Int32(-1))
                                rl.getParams(graphID: 0, fileName: rl.comNames[0]+".json", first: false)
                                rl.realCheck()
                                if rl.real{
                                    
                                    print("real")
                                    rl.updatePorts()
                                    
                                }
                                print(rl.comEnv[0])
                                
                                rl.view = 1
                            }
                        }
                    }.padding()
                }.padding(.horizontal)
                
                HStack{
                    Label("Delete", systemImage: "trash")
                
                    Menu("Delete Experiment...") {
                        ForEach(rl.files, id: \.self){ f in
                            Button(f) {
                                rl.deleteExperiment(fileName: f)
                            }
                        }
                    }.padding()
                }.padding(.horizontal)
                
                HStack{
                    Label("Mocap", systemImage: "camera")
                
                    Menu("Select Experiment...") {
                        ForEach(rl.files, id: \.self){ f in
                            Button(f) {
                                print(f)
                                let ch = Character(".")
                                let result = f.split(separator: ch).map { String($0) }
                                rl.comNames[0] = result[0]
                                rl.newExp = false
                                setPathAndFile(rl.path, rl.comNames[0]+".json", rl.comNames[0]+"-sub.json", Int32(-1))
                                rl.getParams(graphID: 0, fileName: rl.comNames[0]+".json")
                                rl.realCheck()
                                if rl.real{
                                    print("real")
                                    rl.updatePorts()
                                }
                                
                                rl.view = 4
                            }
                        }
                    }.padding()
                }.padding(.horizontal)
                
                
                HStack{
                    
                    Button(action:{
                        rl.comEnv[0] = "cartpolePID"
                        rl.realCheck()
                        setEnv(rl.comEnv[0])
                        rl.view = 3;
                    }, label: {
                        Text("PID").foregroundColor(.green)
                    })
                    
                    Button(action:{
//                        rl.autorun = true
//                        rl.newExp = true
//                        rl.comForm[0] = true
//                        rl.comNames[0] = "a1"
//                        rl.comEnv[0] = "cartpoleClassic"
                        rl.view = 102
                    }, label: {
                        Text("Trim Data").foregroundColor(.green)
                    })
                    
                    
                    Button(action:{
                        rl.comEnv[0] = "trolley"
                        rl.realCheck()
                        setEnv(rl.comEnv[0])
                        rl.updatePorts()
                        rl.view = 5;
                        
                        
                    }, label: {
                        Text("Manual Control").foregroundColor(.green)
                    })
                    
                    Button(action:{
                        rl.view = 101;
                    }, label: {
                        Text("Probe Networks").foregroundColor(.green)
                    })
                    
                    Button(action:{
                        rl.view = 7;
                    }, label: {
                        Text("PI Exhastive Search").foregroundColor(.green)
                    })
                }
                
//                Button(action:{
//                    let appResources = Bundle.main.resourcePath!
//                    let path = String(describing: appResources)
//                    BasicNN(path, rl.curPort)
//                }, label: {
//                    Text("BasicNN").foregroundColor(.green)
//                })
                
                
                
                Spacer()
                
                Image("robot").offset(y: 10)
                              .padding(.bottom, 10)
                              .padding(.leading, 10)
                              .padding(.trailing, 10)
                
                Text("This app uses Deepmind's MuJoCo physics engine, see embeddedRL for the real world reinforcement learning version").padding()
                    .foregroundColor(Color.green)
                    .padding()
                
            }
    }
}
