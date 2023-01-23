//
//  NewView.swift
//  robot
//
//  Created by Kez Smithson Whitehead on 10/05/2022.
//

import SwiftUI

struct NewView: View {
    @EnvironmentObject var rl: RL
    
    @State var autoName:Bool = false
    
    @State var characterset = CharacterSet(charactersIn:
       "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789-"
    )
    @State var validName = true
    
    func validNameCheck(){
        
        if rl.comNames[0].rangeOfCharacter(from: characterset.inverted) != nil {
            if (rl.comNames[0].count > 0 && rl.comNames[0].count < 10){
                validName = true
            }
        } else {
            validName = false
        }
    }
    
    
    var body: some View {
        
        
        Text("Tune you're parameters and click begin!")
            .fontWeight(.ultraLight)
            .foregroundColor(Color.green)
            .multilineTextAlignment(.center)
            .padding(.all,10)
            .font(.title)
            .shadow(color: .green, radius: 10)
            .padding()
        
        Spacer()
        
        Image("robot").offset(y: 10)
            .padding(.bottom, 10)
            .padding(.leading, 10)
            .padding(.trailing, 10)
        Spacer()
        Divider()
        
        HStack{
            VStack(alignment: .leading){
                Form {
                    HStack{
                        Text("Environment")
                        Menu(rl.comEnv[0]) {
                            ForEach(rl.envs, id: \.self){ e in
                                Button(e) {
                                    rl.comEnv[0] = e
                                    rl.realCheck()
                                }
                            }
                        }
                        //                        if autoName{
                        //                            Text("Real world").foregroundColor(.gray)
                        //                        } else {
                        //                            Text("Simulation").foregroundColor(.gray)
                        //                        }
                    }.padding(.leading,-85)
                        .padding(.trailing,90)
                    
                    TextField(text: $rl.comNames[0], prompt: Text("Required e.g. <myExperiment>")) {Text("Name")}.padding(.trailing,90)
                    Stepper("Learning Rate \(rl.comLr[0].formatted())", value: $rl.comLr[0], in: 0.00003...0.03, step:0.00003)
                    
                    Stepper("Seed \(rl.comSeed[0].formatted())", value: $rl.comSeed[0], in: -5...30, step: 1)
                    Stepper("1st Hidden Layer \(rl.comH1[0].formatted())", value: $rl.comH1[0], in: 1...128, step: 1)
                    Stepper("2nd Hidden Layer \(rl.comH2[0].formatted())", value: $rl.comH2[0], in: 1...128, step: 1)
                    Stepper("Epochs \(rl.comEpochs[0].formatted())", value: $rl.comEpochs[0], in: 1...1000, step: 1)
                    Stepper("Minibatch size \(rl.comMinibatch[0].formatted())", value: $rl.comMinibatch[0], in: 8...1280, step: 8)
                    Stepper("Iteration sample size \(rl.comIterSample[0].formatted())", value: $rl.comIterSample[0], in: 50...100000, step: 50)
                    Stepper("Env Step Limit \(rl.comStepLimit[0].formatted())", value: $rl.comStepLimit[0], in: 10...10000, step: 50)
                }.padding()
                Toggle("Adam optimiser", isOn: $rl.comAdam[0]).toggleStyle(.switch).padding(.leading,70)
            }
            Divider()
            
            VStack{
                ParamView()
                Button(action: {
                    rl.goHome()
                }, label: {Label("Scrap experiment", systemImage: "house")
                    .foregroundColor(Color.red)})
                .padding()
            }
        }
        
        HStack{
            
            Button(action:{
                rl.comNames[0] = rl.comEnv[0] + "-" + String(rl.comSeed[0])
            }, label: {
                Label("AutoName", systemImage: "rectangle.and.pencil.and.ellipsis")
            }).padding()
            
            
            
            if (rl.comNames[0].count>0 && rl.comNames[0].rangeOfCharacter(from: characterset.inverted) == nil && rl.comNames[0].count > 0 && rl.comNames[0].count < 20){
                Button(action:{
                    rl.deleteExperiment(fileName:rl.comNames[0]+".json"); // ensure all previous names deleted
                    print("delete previous file")
                    rl.iter = 0
                    setPathAndFile(rl.path, rl.comNames[0]+".json", rl.comNames[0]+"-sub.json", Int32(-1))
                    rl.updateParameters()
                    if rl.real{
                        rl.updatePorts()
                    }
                    rl.view = 1
                }, label: {
                    Label("Begin", systemImage: "brain")
                }).padding()
                
            } else {
                Text("Please enter a valid name between 0 to 20 non special characters").foregroundColor(.red).padding()
            }
        }
    }
}
