from naoqi import ALProxy
import random

class BlackjackBehavior(object):
    def __init__(self, session):
        self.mem = session.service("ALMemory")
        self.tts = session.service("ALTextToSpeech")
        self.mem.subscribeToEvent("Game/LastQR", "BlackjackBehavior", "onNewCard")
        self.mem.subscribeToEvent("Game/LastEmotion", "BlackjackBehavior", "onEmotion")

        # build deck
        self.full_deck = [f"{r} of {s}" for s in ['Hearts','Diamonds','Clubs','Spades']
                                        for r in ['2','3','4','5','6','7','8','9','10','Jack','Queen','King','Ace']]
        self.resetRound()

    def resetRound(self):
        self.deck = self.full_deck[:]
        random.shuffle(self.deck)
        self.player = []
        self.robot  = [self.deck.pop(), self.deck.pop()]
        self.tts.say("New round started, please show me your two cards.")

    def onNewCard(self, key, value, msg):
        card = value  # "10 of Clubs"
        if len(self.player) < 2 and card in self.deck:
            self.player.append(card)
            self.deck.remove(card)
            self.tts.say(f"I see you have {card}.")
            if len(self.player) == 2:
                self.playRobot()

    def onEmotion(self, key, value, msg):
        self.emotion = value  # "happy", "sad", etc.

    def playRobot(self):
        # emotion‐based threshold
        thresh = 17
        if self.emotion == "sad":   thresh = 16
        if self.emotion == "happy": thresh = 18

        # robot hits
        while self.handValue(self.robot) < thresh:
            c = self.deck.pop()
            self.robot.append(c)
            self.tts.say(f"I draw {c}.")

        # dealer (simple sim)
        dealer = []
        while self.handValue(dealer) < 17:
            dealer.append(self.deck.pop())

        # compare
        p_val, r_val, d_val = self.handValue(self.player), self.handValue(self.robot), self.handValue(dealer)
        self.tts.say(f"You: {p_val}, Me: {r_val}, Dealer: {d_val}.")
        if (p_val>21) or (r_val<=21 and r_val>p_val):
            self.tts.say("I win!")
        elif (r_val>21) or (p_val>r_val):
            self.tts.say("You win!")
        else:
            self.tts.say("It's a tie!")

        # start next round
        self.resetRound()

    def handValue(self, hand):
        vals = []
        aces = 0
        for c in hand:
            r = c.split()[0]
            if r in ["Jack","Queen","King"]: vals.append(10)
            elif r=="Ace": vals.append(11); aces+=1
            else: vals.append(int(r))
        tot = sum(vals)
        while tot>21 and aces>0:
            tot -= 10; aces -= 1
        return tot
