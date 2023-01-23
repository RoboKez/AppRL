//
//  TrimView.swift
//  robot
//
//  Created by Kez Smithson Whitehead on 14/12/2022.
//

import SwiftUI

struct TrimView: View {
    @EnvironmentObject var rl: RL
    @StateObject var control = Control()
    @State private var iter:Float = 0
    @State private var loadediter:Int = 0
    @State private var maxiteration:Int32 = 0
    @State private var filename:String = "Select file..."
    @State private var selected:Bool = false
    var body: some View {
    HStack{
        Text("Trim data")
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
                    iter = Float(maxiteration);
                    
                }
            }
                
            
        }.padding()
        
        HStack{
        Stepper("Iteration \(iter.formatted())", value: $iter, in: 0...Float(maxiteration), step: 1)
            

            
            
            
        
        
        Text(("Delete From Iteration: " + String(loadediter)))
            .fontWeight(.ultraLight)
            .foregroundColor(Color.green)
            .multilineTextAlignment(.center)
            .padding(.all,10)
            .font(.title)
            .shadow(color: .green, radius: 10)
            .padding()
        }
        
        if (selected && iter>4){
            Button(action: {
                print("here for delete", iter)
                deleteJsonRange(rl.path, rl.comNames[0]+".json", Int32(iter));
                
            }, label: {Label("Trim Delete", systemImage: "trash")
                .foregroundColor(Color.orange)})
                .padding()
        }
    }
        
}

