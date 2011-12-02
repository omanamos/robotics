using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Speech.Recognition;

using Controller;

namespace VoiceRecog
{
    class VoiceRecogition
    {
        private SpeechRecognitionEngine recogEng;
        private MainController controller;

        public VoiceRecogition(MainController.State state, MainController controller)
        {
            this.initialize(new CommandGrammarBuilder(state, controller.getLibrary()).getGrammar(), controller);
        }

        public VoiceRecogition(Grammar grammar, MainController controller)
        {
            this.initialize(grammar, controller);
        }

        private void initialize(Grammar grammar, MainController controller)
        {
            if (grammar != null)
            {
                this.controller = controller;
                this.recogEng = new SpeechRecognitionEngine();
                recogEng.SetInputToDefaultAudioDevice();

                recogEng.SpeechRecognized += new EventHandler<SpeechRecognizedEventArgs>(speechRecog_success);
                recogEng.SpeechRecognitionRejected += new EventHandler<SpeechRecognitionRejectedEventArgs>(
                    speechRecog_failure);
                recogEng.LoadGrammar(grammar);
                this.start();
            }
        }

        private void speechRecog_success(object sender, SpeechRecognizedEventArgs args)
        {
            Console.WriteLine("Recognized: {0} - {1}", args.Result.Text, args.Result.Confidence);
            controller.processCommand(args.Result.Text);
        }

        private void speechRecog_failure(object sender, SpeechRecognitionRejectedEventArgs args)
        {
            Console.WriteLine("Failed: {0} - {1}", args.Result.Text, args.Result.Confidence);
        }

        public void start()
        {
            recogEng.RecognizeAsync(RecognizeMode.Multiple);
        }

        public void exit()
        {
            try
            {
                recogEng.RecognizeAsyncStop();
            }
            catch (Exception e) { }
        }
    }
}
