//
//  RunningView.swift
//  robot
//
//  Created by Kez Smithson Whitehead on 10/05/2022.
//

import SwiftUI

struct RunningView: View {
    @EnvironmentObject var rl: RL
    @StateObject var live = Live()
    @State var watching = false
    @State var detLimited = true
    @State var prevsp2:Float = 0.0
    @State var decay:Float = 0.01
    @State var iterLimit:Int = 250
    @State var watchedIter:Int = -1
    @State var i_current = 0
    var body: some View {
        
        if (rl.running) {
            if watching{
                GraphView(title: "Live Cart Position",
                          x1: live.xtime, y1: live.ya,
                          x2: live.xtime, y2: live.yat,
                          l1: "Actual", l2: "Target",
                          x_title: "Step", y_title: "Position",
                          minManual: -1.2, maxManual: 1.2).padding()
            }
            stopView
        } else {
            if(rl.real && !rl.portConnected){
                serialView
            } else {
                if(rl.newExp){
                    NewTrainView
                } else {
                    LoadTrainView
                }
            }
        }
        
        
        HStack{
            Text("Iteration: "+String(i_current)).foregroundColor(Color.green)
            Stepper("Limit \(iterLimit.formatted())", value: $iterLimit, in: 10...400, step: 10).padding().foregroundColor(.blue)
            
            Stepper("Start State \(rl.startState.formatted())", value: $rl.startState, in: 0...2, step: 1).padding().foregroundColor(.blue)
            
            Stepper("Noise \(rl.sensorNoise.formatted())", value: $rl.sensorNoise, in: 0...0.1, step: 0.01).padding().foregroundColor(.blue)
            
            
            Stepper("MAAS \(rl.actionFilter.formatted())", value: $rl.actionFilter, in: 0.0...0.5, step: 0.1).padding().foregroundColor(.blue)
            
            Stepper("Decay \(decay.formatted())", value: $decay, in: 0.001...0.5, step: 0.001).padding().foregroundColor(.blue)
            
            Toggle("Anneal", isOn: $rl.annealFilter).toggleStyle(.switch).padding()
        }
    }
}


// Stop button ===============================================================================
extension RunningView {
    private var stopView: some View {
            Button(action: {
                rl.cancelExperiment()
            }, label: {Label("Stop", systemImage: "stop.fill")
                .foregroundColor(Color.red)})
                .padding()
    }
}



// Real robot serial setup ===============================================================================
extension RunningView {
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
            goHomeView
        }
    }
}


// Unsaved (new) Experiment ===============================================================================
extension RunningView {
    private var NewTrainView: some View {
        HStack{
            Button(action: {
                learn(i_start: 0)
            }, label: {Label("Begin Training", systemImage: "brain")
                .foregroundColor(Color.green)})
                .padding()
            goHomeView
        }
    }
}


// Load ===============================================================================
extension RunningView {
    private var LoadTrainView: some View {
        HStack{
            
            goHomeView
            
            
            Button(action: {
                learn(i_start: -1)
            }, label: {Label("Continue Training", systemImage: "brain")
                .foregroundColor(Color.green)})
                .padding()
            
            Button(action: {
                watch(disable_termination: true)
            }, label: {Label("Watch", systemImage: "eyes.inverse")
                .foregroundColor(Color.green)})
                .padding()
            
            Toggle("Limited", isOn: $detLimited).toggleStyle(.switch)
            
        }
    }
}


// Go Home ===============================================================================
extension RunningView {
    private var goHomeView: some View {
        Button(action: {
            rl.goHome()
        }, label: {Label("Home", systemImage: "house")
            .foregroundColor(Color.green)})
            .padding()
    }
}

// Learn ===============================================================================
extension RunningView {
    private func learn(i_start:Int){
        watching=false
        var i_begin = 0
        
        if(!(i_start == 0)){
            i_begin = rl.loadEnv(i: i_start)
            rl.ProbeNet(fileName: rl.comNames[0]+".json")
        } else {
            rl.newExperiment()
            rl.ProbeNet(fileName: rl.comNames[0]+".json", new: true)
        }
        
                
        if(rl.actionFilter<0.0){
            rl.actionFilter = 0.0
        } else if (rl.actionFilter>0.5){
            rl.actionFilter = 0.5
        }
        setActFilter(rl.actionFilter)

        for i in i_begin...i_begin+iterLimit{
            i_current = i
            setWatching(1)
            print("iter:\t", i)
            
            if (rl.running) {
                print(rl.running, (getQuit() == 0))
                
                // Collect data
                batchReset()
                while (getBatchFull() == 0 && getQuit() == 0 && rl.running){
                    rlEpisodeReset(rl.startState)
                    var mycount = 0
                    while (getDone() == 0 && getQuit() == 0 && rl.running){
                        setNoise(rl.sensorNoise)
                        rlStep(0)
                        
//                        print(mycount, ",", rl.getAgentObs()[3])
                        mycount += 1 ;
                    }
                }
                
                if(getQuit() == 0 && !rl.running){break;}
                
                // Det test
                rlEpisodeReset(2)
                while (getDone() == 0 && getQuit() == 0 && rl.running){
                    rlStep(1)
                    setNoise(rl.sensorNoise)
                }
                if(getQuit() == 0 && rl.running){
                    
                    // Save
                    rlSave(Int32(i))
                    
                    // Train Model
//                    rlModel(Int32(i), 5)
                    
                    // PPO Update
//                    ZeroAdamMomentums();
                    UpdatePPO();
                    
                    LinearAnneal();
                    
                    rl.getReturns(graphID: 0, fileName: rl.comNames[0]+".json")
                    
                    rl.ProbeNet(fileName: rl.comNames[0]+".json")
                    
                    rl.newExp = false
                    
                    if(rl.annealFilter){
                        rl.actionFilter -= decay
                    }
                    
                    if(rl.actionFilter<0.0){
                        rl.actionFilter = 0.0
                    } else if (rl.actionFilter>0.5){
                        rl.actionFilter = 0.5
                    }
                    setActFilter(rl.actionFilter)
            
                } else {
                    rl.running=false;
                    break;
                }
            } else {break;}
        }
        rl.closeExperiment()
        
        
    }
}

// Watch ====================================================
extension RunningView {
    private func watch(disable_termination:Bool=true){
        watching=true
        setFramerate(Int32(120))
        let _ = rl.loadEnv(i: watchedIter)
        rl.running = true;
            
        rlEpisodeReset(rl.startState)
        getReturn()
        live.Reset()
        setWatching(1)
        StepLimited(0)
        var counter1 = 0
        while (getQuit() == 0 && rl.running){
            setNoise(rl.sensorNoise)
            rlStep(1)
            print(rl.getAgentObs()[0])
            live.updateGraph(tarPitch: getSetPoint(), curPitch:prevsp2-rl.getAgentObs()[0])
            if (detLimited && Int32(getDone()) != 0){
                print("return:", getReturn())
                break; // lost packet
            }
            
//            print(counter1, ",", getSetPoint2(), ",", prevsp2-rl.getAgentObs()[2])
            counter1 += 1
            prevsp2 = getSetPoint();
        }
        
//        0.9636018
//        0.95347875
        StepLimited(1)
        setWatching(0)
        watching=false
        rl.closeExperiment()
    }
}


