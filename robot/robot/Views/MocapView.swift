//
//  MocapView.swift
//  robot
//
//  Created by Kez Smithson Whitehead on 07/07/2022.
//

import SwiftUI

struct MocapView: View {
    @EnvironmentObject var rl: RL
    @StateObject var live = Live()
    var body: some View {
        HStack{
            Text("Motion Capture")
                .fontWeight(.ultraLight)
                .foregroundColor(Color.green)
                .multilineTextAlignment(.center)
                .padding(.all,10)
                .font(.title)
                .shadow(color: .green, radius: 10)
                .padding()
            
            Spacer()
            
            Button(action: {
                rl.goHome()
            }, label: {Label("Home", systemImage: "house")
                .foregroundColor(Color.green)})
                .padding()
        }
        
        Spacer()
        
        GraphView(title: "Live Pitch",
                  x1: live.xtime, y1: live.ya,
                  x2: live.xtime, y2: live.yat,
                  x3: live.xtime, y3: live.yap,
                  l1: "Actual", l2: "Target", l3: "Predicited",
                  x_title: "Step", y_title: "Angle",
                  minManual: -0, maxManual: 0).padding()
        
        if rl.running{
            Button(action: {
                rl.running = false
            }, label: {Label("Stop", systemImage: "stop.fill")
                .foregroundColor(Color.red)})
                .padding()
        } else {
            Button(action:{
                // actual
                let _ = rl.loadEnv(i: -1)
                rlEpisodeReset(0)
                
                // model predicition
                let _ = rl.MoCaploadAgent(i: -1)
                
                rlMocapReset(rl.comEnv[0])
                rlMocapEpisodeReset()
                
                let tmp_ob = getAgentOb()!
                let ob_dim = getAgentObDim();
                var ob: [Float] = []
                for j in 0...ob_dim-1 {ob.append(tmp_ob[Int(j)])}
                print("Reset ob ", ob)
                live.Reset()
                
                
                while(getQuit() == 0 && rl.running){
                    live.updateGraph3(tarPitch: getSetPoint(), curPitch: getTruePoint(), predPitch: ob[0])
                    rlStep(1)
                    let predicted_ob = rlPredict(UnsafeMutablePointer(&ob))!
                    ob = []
                    for j in 0...ob_dim-1 {ob.append(predicted_ob[Int(j)])}
                    if (checkUserQuit() != 0){break}
                }
                rl.closeExperiment()
                
            }, label: {Label("Test", systemImage: "tuningfork")
                .foregroundColor(Color.green)})
                .padding()
        }
    }
}
