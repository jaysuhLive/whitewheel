# -*- coding: utf-8 -*-

import sys
reload(sys)
sys.setdefaultencoding('utf-8')

from gtts import gTTS
from playsound import playsound

file_name = 'gen.mp3'

text = "hello gys."
tts_ko = gTTS(text=text, lang='ko')
tts_ko.save(file_name)
#playsound(file_name)
# import pyttsx3

# engine = pyttsx3.init()

# engine.setProperty('rate', 200)

# engine.say('무궁화 꽃이 피었습니다')

# engine.runAndWait()
