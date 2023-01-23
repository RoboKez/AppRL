//
//  NetworkView.swift
//  robot
//
//  Created by Kez Smithson Whitehead on 02/12/2022.
//

import SwiftUI

struct NetworkView: View {
    @EnvironmentObject var rl: RL
    @StateObject var control = Control()
    @State private var iter:Float = 0
    @State private var loadediter:Int = 0
    @State private var maxiteration:Int32 = 0
    @State private var filename:String = "Select file..."
    @State private var selected:Bool = false
    var body: some View {
    HStack{
        Text("Probe Policy and Value Networks")
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
        
        Menu(filename) {
            ForEach(rl.files, id: \.self){ f in
                Button(f) {
                    iter = 0
                    loadediter = 0
                    print("max iter is now", maxiteration)
                    print(f)
                    let ch = Character(".")
                    let result = f.split(separator: ch).map { String($0) }
                    rl.comNames[0] = result[0]
                    rl.newExp = false
                    setPathAndFile(rl.path, rl.comNames[0]+".json", rl.comNames[0]+"-sub.json", Int32(-1))
                    rl.getParams(graphID: 0, fileName: rl.comNames[0]+".json", probe: false)
                    
                    let tmpmaxiteration = loadNetwork(Int32(iter), 1)
                    maxiteration = tmpmaxiteration - 1;
                    rl.ProbeNet(fileName: rl.comNames[0]+".json", new: true)
                    selected = true
                    filename = rl.comNames[0] + ".json"
                    
                }
            }
                
            
        }.padding()
        
        HStack{
        Stepper("Iteration \(iter.formatted())", value: $iter, in: 0...Float(maxiteration), step: 1)
        if (selected){
            Button(action: {
                loadediter = Int(iter)
                let tmpmaxiteration = loadNetwork(Int32(iter), 1)
                maxiteration = tmpmaxiteration - 1;
                rl.ProbeNet(fileName: rl.comNames[0]+".json", new: true)
            }, label: {Label("Update", systemImage: "house")
                .foregroundColor(Color.green)})
                .padding()
        }
        
        
        Text(("Probed Iteration: " + String(loadediter)))
            .fontWeight(.ultraLight)
            .foregroundColor(Color.green)
            .multilineTextAlignment(.center)
            .padding(.all,10)
            .font(.title)
            .shadow(color: .green, radius: 10)
            .padding()
        }
        
        
        
        
        
        ScrollView{
        VStack{
        PixelView(data: rl.Policy,
                  xMinID: rl.probeMinP,
                  xMaxID: rl.probeMaxP,
                  yMinID: rl.probeMinP,
                  yMaxID: rl.probeMaxP,
                  title:"Policy",
                  Xlabel: "Pitch",
                  Ylabel: "Gyro")
        
        PixelView(data: rl.Values,
                  xMinID: rl.probeMinV,
                  xMaxID: rl.probeMaxV,
                  yMinID: rl.probeMinV,
                  yMaxID: rl.probeMaxV,
                  title:"Value",
                  Xlabel: "Pitch",
                  Ylabel: "Gyro")
            
        
            PixelView(data: rl.Policy2,
                      xMinID: rl.probeMinP,
                      xMaxID: rl.probeMaxP,
                      yMinID: rl.probeMinP,
                      yMaxID: rl.probeMaxP,
                      title:"Policy",
                      Xlabel: "Cart Pos",
                      Ylabel: "Cart Vel")
            
            PixelView(data: rl.Values2,
                      xMinID: rl.probeMinV,
                      xMaxID: rl.probeMaxV,
                      yMinID: rl.probeMinV,
                      yMaxID: rl.probeMaxV,
                      title:"Value",
                      Xlabel: "Cart Pos",
                      Ylabel: "Cart Vel")
        }
        }
        
   }
}

