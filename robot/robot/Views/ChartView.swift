//
//  ChartView.swift
//  robot
//
//  Created by Kez Smithson Whitehead on 10/05/2022.
//

import SwiftUI

struct TextEditingView: View {
    @EnvironmentObject var rl: RL
    var body: some View {
        TextEditor(text: $rl.curTitle)
            .foregroundColor(Color.gray)
            .border(Color.gray)
            .font(.subheadline)
    }
}

struct ChartView: View {
    @EnvironmentObject var rl: RL
    @State var data:[[Float]] = [[]]
    var body: some View {
        
        HStack{
    
                ScrollView {
                    
                    Text(rl.curTitle)
                    .fontWeight(.ultraLight)
                    .foregroundColor(Color.green)
                    .multilineTextAlignment(.center)
                    .padding(.all,10)
                    .font(.title)
                    Divider()
                
                    plotsView
                    Text("Select data for comparison")
                        .fontWeight(.ultraLight)
                        .foregroundColor(Color.green)
                        .multilineTextAlignment(.center)
                        .padding(.all,10)
                        .font(.title)
                    
                    
                    compareView
                    
                    
                    
                    HStack{
                        Text("Edit Title")
                        TextEditingView()
                    }.padding()
                }
            
            Divider()
            ScrollView{
                ParamView()
//                if(rl.comEnv[0] == "cartpole" || rl.comEnv[0] == "trolley"){
                
                    PixelView(data: rl.Policy,
                              xMinID: rl.probeMinP,
                              xMaxID: rl.probeMaxP,
                              yMinID: rl.probeMinP,
                              yMaxID: rl.probeMaxP,
                              title:"Policy",
                              Xlabel: "P",
                              Ylabel: "I")
                    
                    PixelView(data: rl.Values,
                              xMinID: rl.probeMinV,
                              xMaxID: rl.probeMaxV,
                              yMinID: rl.probeMinV,
                              yMaxID: rl.probeMaxV,
                              title:"Value",
                              Xlabel: "P",
                              Ylabel: "I")
                
if(rl.comEnv[0] == "cartpoleClassic"){
                PixelView(data: rl.Policy2,
                          xMinID: rl.probeMinP,
                          xMaxID: rl.probeMaxP,
                          yMinID: rl.probeMinP,
                          yMaxID: rl.probeMaxP,
                          title:"Policy",
                          Xlabel: "Cart P",
                          Ylabel: "Vart V")
                
                PixelView(data: rl.Values2,
                          xMinID: rl.probeMinV,
                          xMaxID: rl.probeMaxV,
                          yMinID: rl.probeMinV,
                          yMaxID: rl.probeMaxV,
                          title:"Value",
                          Xlabel: "Cart P",
                          Ylabel: "Cart V")
                    
                }
            }
        }
    }
}


    extension ChartView {
        private var plotsView: some View {
            
            ScrollView {
            GraphView(title: "Stocastic",
                      x1: rl.x1, y1: rl.y1s,
                      x2: rl.x2, y2: rl.y2s,
                      x3: rl.x3, y3: rl.y3s,
                      x4: rl.x4, y4: rl.y4s,
                      x5: rl.x5, y5: rl.y5s,
                      l1:rl.comNames[0],
                      l2:rl.comNames[1],
                      l3:rl.comNames[2],
                      l4:rl.comNames[3],
                      l5:rl.comNames[4],
                      c1: rl.comColour[0],
                      c2: rl.comColour[1],
                      c3: rl.comColour[2],
                      c4: rl.comColour[3],
                      c5: rl.comColour[4]
            ).padding()
                
            Divider()

            GraphView(title: "Deterministic",
                      x1: rl.x1, y1: rl.y1d,
                      x2: rl.x2, y2: rl.y2d,
                      x3: rl.x3, y3: rl.y3d,
                      x4: rl.x4, y4: rl.y4d,
                      x5: rl.x5, y5: rl.y5d,
                      l1:rl.comNames[0],
                      l2:rl.comNames[1],
                      l3:rl.comNames[2],
                      l4:rl.comNames[3],
                      l5:rl.comNames[4],
                      c1: rl.comColour[0],
                      c2: rl.comColour[1],
                      c3: rl.comColour[2],
                      c4: rl.comColour[3],
                      c5: rl.comColour[4]
            ).padding()
        }
        }
    }


    extension ChartView {
        private var compareView: some View {
            
            ScrollView{
                
                // d2 users choose
                HStack{
                    Text("d2:").foregroundColor(rl.comColour[1])
                    Menu(rl.comNames[1]) {
                        Button("None") {
                            rl.comNames[1] = "None"
                            rl.x2 = []
                            rl.y2s = []
                            rl.y2d = []
                            rl.comForm[1] = false
                        }
                        ForEach(rl.files, id: \.self){ f in
                            if (f != rl.comNames[0]+".json" && f != rl.comNames[1]+".json" && f != rl.comNames[2]+".json" && f != rl.comNames[3]+".json" && f != rl.comNames[4]+".json"){
                                Button(f) {
                                    rl.getParams(graphID: 1, fileName: f, probe:false)
                                }
                            }
                        }
                    }
                }.padding(.horizontal)
                
                
                // d3 users choose
                HStack{
                    Text("d3:").foregroundColor(rl.comColour[2])
                    Menu(rl.comNames[2]) {
                        Button("None") {
                            rl.comNames[2] = "None"
                            rl.x3 = []
                            rl.y3s = []
                            rl.y3d = []
                            rl.comForm[2] = false
                        }
                        ForEach(rl.files, id: \.self){ f in
                            if (f != rl.comNames[0]+".json" && f != rl.comNames[1]+".json" && f != rl.comNames[2]+".json" && f != rl.comNames[3]+".json" && f != rl.comNames[4]+".json"){
                                Button(f) {
                                    rl.getParams(graphID: 2, fileName: f, probe:false)
                                }
                            }
                        }
                    }
                }.padding(.horizontal)
                
                // d4 users choose
                HStack{
                    Text("d4:").foregroundColor(rl.comColour[3])
                    Menu(rl.comNames[3]) {
                        Button("None") {
                            rl.comNames[3] = "None"
                            rl.x4 = []
                            rl.y4s = []
                            rl.y4d = []
                            rl.comForm[3] = false
                        }
                        ForEach(rl.files, id: \.self){ f in
                            if (f != rl.comNames[0]+".json" && f != rl.comNames[1]+".json" && f != rl.comNames[2]+".json" && f != rl.comNames[3]+".json" && f != rl.comNames[4]+".json"){
                                Button(f) {
                                    rl.getParams(graphID: 3, fileName: f, probe:false)
                                }
                            }
                        }
                    }
                }.padding(.horizontal)
                
                // d5 users choose
                HStack{
                    Text("d5:").foregroundColor(rl.comColour[4])
                    Menu(rl.comNames[4]) {
                        Button("None") {
                            rl.comNames[4] = "None"
                            rl.x5 = []
                            rl.y5s = []
                            rl.y5d = []
                            rl.comForm[4] = false
                    }
                    ForEach(rl.files, id: \.self){ f in
                        if (f != rl.comNames[0]+".json" && f != rl.comNames[1]+".json" && f != rl.comNames[2]+".json" && f != rl.comNames[3]+".json" && f != rl.comNames[4]+".json"){
                            Button(f) {
                                rl.getParams(graphID: 4, fileName: f, probe:false)
                            }
                        }
                    }
                }
            }.padding(.horizontal)
        }
    }
}
