//
//  ParamView.swift
//  robot
//
//  Created by Kez Smithson Whitehead on 13/05/2022.
//

import SwiftUI

struct ParamView: View {
    @EnvironmentObject var rl: RL
    var body: some View {
        ScrollView{
            ForEach(rl.comForm.indices, id: \.self){ index in
                if rl.comForm[index]{
                    Form {
                        Text("Name:\t\t\t" + rl.comNames[index])
                        Text("Environment:\t\t" + rl.comEnv[index])
                        Text("Env Step Limit:\t" + String(rl.comStepLimit[index].formatted()))
                        Text("Learning Rate:\t" + String(rl.comLr[index]))
                        Text("Seed:\t\t\t" + String(rl.comSeed[index]))
                        Text("Hidden Layers:\t" + String(rl.comH1[index]) + ", " + String(rl.comH2[index]))
                        Text("Epochs:\t\t\t" + String(rl.comEpochs[index]))
                        Text("Minibatch size:\t" + String(rl.comMinibatch[index]))
                        Text("Iter sample size:\t" + String(rl.comIterSample[index]))
                        Text("Adam optimiser:\t" + String(rl.comAdam[index]))
                    }
                    .padding()
                    .border(rl.comColour[index])
                    .padding()
                }
            }
        }
    }
}
