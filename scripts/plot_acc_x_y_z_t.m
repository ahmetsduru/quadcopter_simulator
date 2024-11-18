function plot_acc_x_y_z_t(log_folder)
    % Dosya yollarını belirleme
    actual_file_path = fullfile(log_folder, 'states.txt');
    ref_file_path = fullfile(log_folder, 'ref_pose.txt');

    % Verileri okuma
    actual_data = readtable(actual_file_path, 'Delimiter', ',');
    ref_data = readtable(ref_file_path, 'Delimiter', ',');

    % Timestamp ve ivme verilerini ayırma
    actual_timestamp = actual_data{:, 1};  % 1. kolon: timestamp
    actual_acc_x = actual_data{:, 8};      % 8. kolon: actual_acc_x
    actual_acc_y = actual_data{:, 9};      % 9. kolon: actual_acc_y
    actual_acc_z = actual_data{:, 10};     % 10. kolon: actual_acc_z

    ref_timestamp = ref_data{:, 1};        % 1. kolon: timestamp
    ref_acc_x = ref_data{:, 8};            % 8. kolon: ref_acc_x
    ref_acc_y = ref_data{:, 9};            % 9. kolon: ref_acc_y
    ref_acc_z = ref_data{:, 10};           % 10. kolon: ref_acc_z

    % Timestamp verilerini normalize etme
    if ~isempty(actual_timestamp) && ~isempty(ref_timestamp)
        actual_timestamp = actual_timestamp - actual_timestamp(1); % İlk timestamp'i çıkar
        ref_timestamp = ref_timestamp - ref_timestamp(1);          % İlk timestamp'i çıkar
    end

    % Eğer veriler doğruysa, subplot grafikleri çizelim
    if ~isempty(actual_timestamp) && ~isempty(actual_acc_x) && ~isempty(actual_acc_y) && ~isempty(actual_acc_z) ...
            && ~isempty(ref_timestamp) && ~isempty(ref_acc_x) && ~isempty(ref_acc_y) && ~isempty(ref_acc_z)
        
        figure;

        % İlk subplot: a_acc_x ve ref_acc_x vs. Normalized Timestamp
        subplot(3, 1, 1);
        plot(actual_timestamp, actual_acc_x, 'r', 'LineWidth', 1.5);
        hold on;
        plot(ref_timestamp, ref_acc_x, 'k:', 'LineWidth', 1.5);
        xlabel('Time (s)');
        ylabel('Acceleration x (m/s^2)');
        title('Desired and Actual Acceleration-Time');
        legend('Actual Acc. x', 'Desired Acc. x');
        grid on;
        hold off;

        % İkinci subplot: a_acc_y ve ref_acc_y vs. Normalized Timestamp
        subplot(3, 1, 2);
        plot(actual_timestamp, actual_acc_y, 'r', 'LineWidth', 1.5);
        hold on;
        plot(ref_timestamp, ref_acc_y, 'k:', 'LineWidth', 1.5);
        xlabel('Time (s)');
        ylabel('Acceleration y (m/s^2)');
        title('Desired and Actual Acceleration-Time');
        legend('Actual Acc. y', 'Desired Acc. y');
        grid on;
        hold off;

        % Üçüncü subplot: a_acc_z ve ref_acc_z vs. Normalized Timestamp
        subplot(3, 1, 3);
        plot(actual_timestamp, actual_acc_z, 'r', 'LineWidth', 1.5);
        hold on;
        plot(ref_timestamp, ref_acc_z, 'k:', 'LineWidth', 1.5);
        xlabel('Time (s)');
        ylabel('Acceleration z (m/s^2)');
        title('Desired and Actual Acceleration-Time');
        legend('Actual Acc. z', 'Desired Acc. z');
        grid on;
        hold off;

    else
        disp('Veriler boş olabilir, lütfen dosya formatını kontrol edin.');
    end
end
