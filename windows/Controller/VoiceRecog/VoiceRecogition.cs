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
            this.initialize(new CommandGrammarBuilder(state, controller.getLibrary()).getGrammar(), controller, true);
        }

        public VoiceRecogition(Grammar grammar, MainController controller)
        {
            this.initialize(grammar, controller, false);
        }

        private void initialize(Grammar grammar, MainController controller, bool start)
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
                if (start)
                {
                    this.start();
                }
            }
        }

        private void speechRecog_success(object sender, SpeechRecognizedEventArgs args)
        {
            if (!args.Result.Text.Equals("nao abort") || args.Result.Confidence > 1.0f)
            {
                Console.WriteLine("Recognized: {0} - {1}", args.Result.Text, args.Result.Confidence);
                controller.processCommand(args.Result.Text);
            }
            else
            {
                Console.WriteLine("Failed: {0} - {1}", args.Result.Text, args.Result.Confidence);
            }
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
