function [] = realTimePlotter()

  % Set up figure, get properties, and label
  figure
  h = animatedline;
  ax = gca;
  ax.YLim = [-5 5];
  xlabel('Time (s)')
  ylabel('Acceleration [g]')

  % Start a timer
  tic
  startTime = toc

  % Loop for 30 secounds
  while toc < 30
    % Some data...

    % Get current time and add to animation
    t = toc - startTime;
    addpoints(h,t,v)

    % Update axes
    ax.XLim = [t t+15];
    drawnow
  end

end
