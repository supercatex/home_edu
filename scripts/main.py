#!/usr/bin/env python
# -*- coding: utf-8 -*

# Old questions keyword structure
question =	{
  "handsome person guy": "I think that Justin Trudeau is very handsome",
  "time zones": "Canada spans almost ten million square km and comprises six time zones",
  "longest street": "Yonge Street in Ontario is the longest street in the world",
  "Yonge Street": "Yonge street is almost two thousand km, starting at Lake Ontario, and running north to the Minnesota border",
  "1915": "The bear cub was named Winnipeg. It inspired thestories of Winnie the Pooh",
  "blackberry": "It was developed in Ontario, at Research In Motion's Waterloo offices.",
  "largest": "The Big Nickel in Sudbury, Ontario. It is nine meters in diameter.",
  "first time": "The first time that the USA invaded Canada was in seventeen seventy five.",
  "second time": "The USA invaded Canada a second time in sighteen and twelve.",
  "medals": "Canada does! With fourteen Golds at the twenty ten VancouverWinter Olympics.",
  "term": "Sandy Gardiner, a journalist of the Ottawa Journal.",
  "named": "French explorers misunderstood the local nativeword Kanata, which means village.",
  "Mounted Police": "The Mounted Police was formed in eighteen seventy three.",
  "Royal Canadian police": "In nighteen twenty, when The Mounted Police merged with the Dominion Police.",
  "RCMP": "Today, the RCMP has close to thirty thousand members.",
  "What else": "Montreal is often called the City of Saints or the City of a Hundred Bell Towers.",
  "located": "The Hotel de Glace is in Quebec.",
  "ice": "The Hotel de Glace requires about four hundred tons of ice.",
  "snow": "Every year, twelve thousand tons of snow are used for The Hotel de Glace.",
  "summer": "No. Every summer it melts away, only to be rebuilt the following winter.",
  "where only desert": "Canada's only desert is British Columbia.",
  "big only desert texas": "The British Columbia desert is only fifthteen miles long.",
  "male": "Leonard Cohen, Keanu Reeves, and Jim Carrey.",
  "female": "Celine Dion, Pamela Anderson, and Avril Lavigne.",
  "origin": "Comic Sans is based on Dave Gibbons' lettering in the Watchmen comic books",
  "nanobot": "The smallest robot possible is called a nanobot.",
  "small": "A nanobot can be less than one-thousandth of amillimeter.",
  "award": "The Academy thought that Tron cheated by using computers.",
  "which hard drive": "The IBM three o five RAMAC.",
  "when hard drive": "The IBM three o five RAMAC was launched in nighteen fifthty six.",
  "big hard disk": "The IBM three o five RAMAC hard disk weighed over a ton and stored five megabytes of data.",
  "CAPTCHA for": "CAPTCHA is an acronym for Completely Automated Public Turing test to tell Computers and Humans Apart",
  "bug": "The first actual computer bug was a dead moth stuck in a Harvard Mark two.",
  "Mars": "There are four robots on Mars include Sojourner, Spirit,Opportunity, and Curiosity. Three more crashed on landing.",
  "android": "Professor Kevin Warwick uses chips in his arm to operate doors, a robotic hand, and a wheelchair.",
  "mechanical": "A robot sketch made by Leonardo DaVinci.",
  "pass": "Some people think it was IBM Watson, but it was Eugene, a computer designed at England's University of Reading",
  "state": "Moravec's paradox states that a computer can crunch numbers like Bernoulli, but lacks a toddler’s motor skills.",
  "engineering": "It is when you need to load an AI with enough knowledge to start learning.",
  "worried impact humanity": "I don't know. He should worry more about the people’s impact on humanity.",
  "think": "No. Humans are the real threat to humanity.",
  "chatbot": "A chatbot is an A.I. you put in customer service to avoid paying salaries.",
  "safe": "Yes. Car accidents are product of human misconduct.",
  "invented compiler": "Grace Hoper. She wrote it in her spare time.",
  "C Programming Language": "C was invented by Dennis MacAlistair Ritchie.",
  "Python Programming Language": "Python was invented by Guido van Rossum.",
  "Mark": "Sure. I've never seen him drink water.",
  "Apple": "My lord and master Steve Wozniak.",
  "programmer": "Ada Lovelace.",
  "PDF": "Adobe Wan Kenobi.",
}

# New question keyword structure
data = []


def load_data(path):
    situation = 0
    file = open(path)
    
    for ds in file.readlines():
        if ds == "\n":
            situation = 1
            continue
        
        elif situation == 1:
            data.append({
                "question": "",
                "answer": "",
                "keyword": []
            })
            data[-1]["question"] = ds.strip()
            situation = 2
        
        elif situation == 2:
            data[-1]["answer"] = ds.strip()
            situation = 3
        
        elif situation == 3:
            keyword = ds.strip().split(",")
            for k in range(len(keyword)):
                keyword[k] = keyword[k].strip()
            data[-1]["keyword"].append(keyword)
                
    file.close()
    return data
    

def answer_question_from_data(text, database):
    for d in database:
        t_and = True
        for keys in d["keyword"]:
            t_or = False
            for key in keys:
                if key[0] != "!":
                    if key.lower() in text.lower().strip().split(" "):
                        t_or = True
                        break
    
                    else:
                        t_or = False
                elif key[0] == "!":
                    if not key.lower().replace("!", "") in text.lower().strip().split(" "):
                        t_or = True
                        break
    
                    else:
                        t_or = False
                
            if not t_or:
                t_and = False
    
        if t_and:
            return d
    
    return {"question": "", "answer": "", "keyword": []}
            

if __name__ == '__main__':
    import speech_recognition as sr
    import pyttsx3
    
    engine = pyttsx3.init()
    
    engine.setProperty("rate", 130)
    engine.setProperty("volume", 1.0)
    
    voices = engine.getProperty("voices")
    voice = voices[10]
    engine.setProperty('voice', voice.id)

    _r = sr.Recognizer()

    data = load_data()
    
    while True:
        with sr.Microphone() as source:
            print("Preparing:")
            _r.adjust_for_ambient_noise(source)
            print("Ready....")
            audio = _r.listen(source)
        question = _r.recognize_google(audio, language="en-US")
        answer = answer_question_from_data(question, data)
        print("OK")
        engine.say("You said: %s" % answer["question"])
        print(answer["question"])
        engine.say(answer["answer"])
        print(answer["answer"])
        engine.runAndWait()
