%%
ur5 = ur5_interface();

pause(10);

%% Mofifiable variables, see README.txt

wordToWrite = 'go h';
K = 0.75;
mode = 'rviz'; %'mplot' or 'rviz'

%%
word = phrase(wordToWrite, ur5, K);

%%
if strcmp(mode, 'rviz')
    word.homenoRR();
    pause(10);
end

%%
word.draw(mode);