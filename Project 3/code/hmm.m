clc
clear all
close all

%% If training is set to 'true', thains the new hmm model, if 'false', loads a saved model
training = false;

%% different gestures in the dataset in alphabetical order
gestures = cellstr(['beat3 ';'beat4 ';'circle';'eight ';'inf   ';'wave  ']);

%% Train a new model or load a model
if training
    %% Train files are present in the same code folder inside Train directory
    model = train_hmm('Train/',gestures,12,80);    
else
    load('model.mat');
end

%% Test files are present in the same code folder inside Test directory
[predictions, confidence] = test_hmm('Test/',gestures,model);