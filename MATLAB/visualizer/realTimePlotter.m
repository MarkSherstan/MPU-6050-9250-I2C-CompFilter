function [] = realTimePlotter()

  % Set up figure, get properties, and label
  figure
  h = animatedline;
  ax = gca;
  ax.YLim = [0 10];
  xlabel('Time (s)')
  ylabel('Acceleration [g]')

  % Start a timer
  tic
  startTime = toc;

  % Loop for 20 secounds
  while toc < 20
    % Random data
    v = randi(4);

    % Get current time and add to animation
    t = toc - startTime;
    addpoints(h,t,v)

    % Update axes
    if t < 5
        ax.XLim = [0 10];
        drawnow
    else
        ax.XLim = [t-5 t+5];
        drawnow
    end

  end
